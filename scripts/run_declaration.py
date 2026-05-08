#!/usr/bin/env python3
"""JSON-declaration batch runner for the swarm simulator.

Reads a declaration file describing a shared default scenario plus a list of
per-simulation overrides, runs each scenario via Docker, parses the
end-of-simulation metrics block, and prints/saves averages across all runs.

The formation-control algorithm is selected once at the CLI
(`--algorithm=centroid|weighted`) and passed through to every run; the JSON
stays algorithm-agnostic so a caller can loop over algorithms without
rewriting declarations.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import shlex
import statistics
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


REPO = Path(__file__).resolve().parent.parent
NUM_DRONES = 20
NUM_ANCHORS = 18

DEFAULT_IMAGE = "swarm-sim"
DEFAULT_TIMEOUT_SECONDS = 1200
DEFAULT_PARAMS = {
    "sim_seconds": 60.0,
}

METRIC_SPECS = [
    ("tri_avg", "Trilateration avg error (m)", 1.0),
    ("tri_max", "Trilateration max error (m)", 1.0),
    ("rx_pos_update_total", "RX POS_UPDATE total", 1.0),
    ("rx_pos_ack_total", "RX POS_ACK total", 1.0),
    ("rx_help_proxy_total", "RX HELP_PROXY total", 1.0),
    ("rx_flood_total", "RX FLOOD total", 1.0),
    ("rx_neighbor_total", "RX NEIGHBOR total", 1.0),
    ("rx_uwb_beacon_total", "RX UWB_BEACON total", 1.0),
    ("dist_helper_total", "Distance helper total (m)", 1.0),
    ("dist_lost_total", "Distance lost total (m)", 1.0),
    ("dist_all_total", "Distance all total (m)", 1.0),
    ("healing_avg_s", "Healing latency avg (s)", 1.0),
    ("healing_p95_s", "Healing latency p95 (s)", 1.0),
    ("healing_max_s", "Healing latency max (s)", 1.0),
    ("return_avg_s", "Return latency avg (s)", 1.0),
    ("return_p95_s", "Return latency p95 (s)", 1.0),
    ("return_max_s", "Return latency max (s)", 1.0),
    ("recovery_rate_pct", "Recovery rate (%)", 1.0),
    ("return_path_eff_avg", "Return path efficiency avg", 1.0),
    ("return_path_eff_max", "Return path efficiency max", 1.0),
    ("boundary_avg_m", "Boundary distance avg (m)", 1.0),
    ("boundary_stddev_m", "Boundary distance stddev (m)", 1.0),
    ("rearm_total", "Rearm events total", 1.0),
    ("station_drift_avg_m", "Station drift avg (m)", 1.0),
    ("station_drift_max_m", "Station drift max (m)", 1.0),
    ("final_coverage_rate_pct", "Final coverage rate (%)", 1.0),
    ("failures_executed", "Failures executed", 1.0),
    ("base_failures_executed", "Base failures executed", 1.0),
]

PARAM_TO_CLI = {
    "sim_seconds": "simSeconds",
    "max_range_meters": "maxRangeMeters",
    "k_att": "kAtt",
    "k_rep": "kRep",
    "d_safe": "dSafe",
    "v_max": "vMax",
    "drone_weight_kg": "droneWeightKg",
    "uwb_noise_std_dev": "uwbNoiseStdDev",
}

PARAM_KEY_ALIASES = {
    "sim_seconds": "sim_seconds",
    "simseconds": "sim_seconds",
    "max_range_meters": "max_range_meters",
    "maxrangemeters": "max_range_meters",
    "k_att": "k_att",
    "katt": "k_att",
    "k_rep": "k_rep",
    "krep": "k_rep",
    "d_safe": "d_safe",
    "dsafe": "d_safe",
    "v_max": "v_max",
    "vmax": "v_max",
    "drone_weight_kg": "drone_weight_kg",
    "droneweightkg": "drone_weight_kg",
    "uwb_noise_std_dev": "uwb_noise_std_dev",
    "uwbnoisestddev": "uwb_noise_std_dev",
}

VALID_ALGORITHMS = ("centroid", "weighted")

TRI_RE = re.compile(
    r"Trilateration:\s*min=([\d.eE+-]+)m\s+max=([\d.eE+-]+)m\s+avg=([\d.eE+-]+)m"
)

PACKET_ROW_RE = re.compile(
    r"^\s*(\d+)\s+(helper|lost)\s+\S+\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$",
    re.MULTILINE,
)

DISTANCE_TOTAL_RE = re.compile(
    r"Distance total:\s*helper=([\d.eE+-]+)m\s+lost=([\d.eE+-]+)m\s+all=([\d.eE+-]+)m"
)

# Stats-bearing lines collapse to just `drones=N` when N=0 (no avg/p95/max).
# All following regexes mark the stats group as optional so parsing a degenerate
# run (nothing completed) still succeeds — those metrics simply end up None.
HEALING_RE = re.compile(
    r"Healing:\s*drones=(\d+)"
    r"(?:\s+avg=([\d.eE+-]+)s\s+p50=([\d.eE+-]+)s\s+p95=([\d.eE+-]+)s\s+max=([\d.eE+-]+)s)?"
)

RETURN_LATENCY_RE = re.compile(
    r"Return:\s*drones=(\d+)"
    r"(?:\s+avg=([\d.eE+-]+)s\s+p50=([\d.eE+-]+)s\s+p95=([\d.eE+-]+)s\s+max=([\d.eE+-]+)s)?"
)

RECOVERY_RE = re.compile(
    r"Recovery:\s*lost=(\d+)\s+armed=(\d+)\s+started=(\d+)\s+completed=(\d+)\s+rate=([\d.eE+-]+)%"
)

RETURN_PATH_RE = re.compile(
    r"ReturnPath:\s*drones=(\d+)"
    r"(?:\s+avg=([\d.eE+-]+)\s+p95=([\d.eE+-]+)\s+max=([\d.eE+-]+))?"
)

BOUNDARY_RE = re.compile(
    r"Boundary:\s*drones=(\d+)"
    r"(?:\s+avg=([\d.eE+-]+)m\s+stddev=([\d.eE+-]+)m\s+target=([\d.eE+-]+)m)?"
)

REARM_RE = re.compile(
    r"Rearm:\s*drones=(\d+)\s+total=(\d+)\s+max=(\d+)"
)

STATION_DRIFT_RE = re.compile(
    r"StationDrift:\s*drones=(\d+)"
    r"(?:\s+avg=([\d.eE+-]+)m\s+p95=([\d.eE+-]+)m\s+max=([\d.eE+-]+)m)?"
)

FINAL_COVERAGE_RE = re.compile(
    r"FinalCoverage:\s*covered=(\d+)\s+total=(\d+)\s+rate=([\d.eE+-]+)%"
    r"\s+hops1=(\d+)\s+hops2=(\d+)\s+hops3plus=(\d+)\s+lost=(\d+)"
    r"(?:\s+dead=(\d+))?"
)

FAILURES_RE = re.compile(r"(?:^|\s)Failures:\s*scheduled=(\d+)\s+executed=(\d+)")
BASE_FAILURES_RE = re.compile(r"BaseFailures:\s*scheduled=(\d+)\s+executed=(\d+)")

DRONE_KEY_RE = re.compile(r"^drone(\d+)$", re.IGNORECASE)
ANCHOR_KEY_RE = re.compile(r"^(anchor|uwb_anchor)(\d+)$", re.IGNORECASE)


class DeclarationError(ValueError):
    """Raised when the declaration file is invalid."""


Vec3 = tuple[float, float, float]


@dataclass
class Scenario:
    name: str
    base_station: Vec3 | None
    drones: dict[int, Vec3]
    anchors: dict[int, Vec3]
    params: dict[str, float]
    failures: list[tuple[int, float]] = field(default_factory=list)
    # v1 multi-base: up to 3 base positions (index 1-based in scenario file).
    bases: list[Vec3] = field(default_factory=list)
    # v1 pinning (v2 silently ignores it but we still round-trip it).
    drone_base: dict[int, int] = field(default_factory=dict)
    # v2: mid-sim base kills (base_idx 1-based, at_seconds).
    base_failures: list[tuple[int, float]] = field(default_factory=list)


@dataclass
class ScenarioDefaults:
    base_station: Vec3 | None
    drones: dict[int, Vec3]
    anchors: dict[int, Vec3]
    params: dict[str, float]
    failures: list[tuple[int, float]] = field(default_factory=list)
    bases: list[Vec3] = field(default_factory=list)
    drone_base: dict[int, int] = field(default_factory=dict)
    base_failures: list[tuple[int, float]] = field(default_factory=list)


@dataclass
class RunRecord:
    run_id: str
    name: str
    status: str
    return_code: int
    duration_seconds: float
    metrics: dict[str, float | int | None]
    params: dict[str, float]
    scenario_file: str
    stdout_file: str
    stderr_file: str


# ── Declaration parsing ────────────────────────────────────────────────────

def canonical_param_key(raw: str) -> str:
    return re.sub(r"[^a-z0-9_]", "", raw.strip().lower().replace("-", "_"))


def normalize_param_name(raw: str, where: str) -> str:
    key = canonical_param_key(raw)
    if key not in PARAM_KEY_ALIASES:
        allowed = ", ".join(sorted(PARAM_TO_CLI))
        raise DeclarationError(f"{where}: unsupported parameter '{raw}', allowed: {allowed}")
    return PARAM_KEY_ALIASES[key]


def as_float(value: Any, where: str) -> float:
    if isinstance(value, bool):
        raise DeclarationError(f"{where}: expected a number, got boolean")
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError as exc:
            raise DeclarationError(f"{where}: expected a number, got '{value}'") from exc
    raise DeclarationError(f"{where}: expected a number, got {type(value).__name__}")


def as_int(value: Any, where: str) -> int:
    if isinstance(value, bool):
        raise DeclarationError(f"{where}: expected an integer, got boolean")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        text = value.strip()
        if text.isdigit() or (text.startswith("-") and text[1:].isdigit()):
            return int(text)
    raise DeclarationError(f"{where}: expected an integer")


def parse_vec3(value: Any, where: str) -> Vec3:
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
        if len(parts) != 3:
            raise DeclarationError(f"{where}: expected vector formatted as x,y,z")
        return (as_float(parts[0], where), as_float(parts[1], where), as_float(parts[2], where))

    if isinstance(value, dict):
        if not {"x", "y", "z"}.issubset(value):
            raise DeclarationError(f"{where}: dict vector must contain keys x, y, z")
        return (
            as_float(value["x"], f"{where}.x"),
            as_float(value["y"], f"{where}.y"),
            as_float(value["z"], f"{where}.z"),
        )

    if isinstance(value, (list, tuple)):
        if len(value) != 3:
            raise DeclarationError(f"{where}: vector list must have exactly 3 values")
        return (
            as_float(value[0], f"{where}[0]"),
            as_float(value[1], f"{where}[1]"),
            as_float(value[2], f"{where}[2]"),
        )

    raise DeclarationError(f"{where}: expected vector as list, dict, or x,y,z string")


def parse_drone_index(raw_key: Any, where: str) -> int:
    if isinstance(raw_key, bool):
        raise DeclarationError(f"{where}: invalid drone key type")
    if isinstance(raw_key, int):
        idx = raw_key
    else:
        text = str(raw_key).strip()
        if text.isdigit():
            idx = int(text)
        else:
            match = DRONE_KEY_RE.match(text)
            if not match:
                raise DeclarationError(f"{where}: invalid drone key '{raw_key}'")
            idx = int(match.group(1))
    if idx < 1 or idx > NUM_DRONES:
        raise DeclarationError(f"{where}: drone index {idx} out of range 1..{NUM_DRONES}")
    return idx


def parse_anchor_index(raw_key: Any, where: str) -> int:
    if isinstance(raw_key, bool):
        raise DeclarationError(f"{where}: invalid anchor key type")
    if isinstance(raw_key, int):
        idx = raw_key
    else:
        text = str(raw_key).strip()
        if text.isdigit():
            idx = int(text)
        else:
            match = ANCHOR_KEY_RE.match(text)
            if not match:
                raise DeclarationError(f"{where}: invalid anchor key '{raw_key}'")
            idx = int(match.group(2))
    if idx < 1 or idx > NUM_ANCHORS:
        raise DeclarationError(f"{where}: anchor index {idx} out of range 1..{NUM_ANCHORS}")
    return idx


def parse_position_block(value: Any, where: str, parse_index) -> dict[int, Vec3]:
    if value is None:
        return {}

    out: dict[int, Vec3] = {}
    if isinstance(value, list):
        for i, item in enumerate(value, start=1):
            idx = parse_index(i, f"{where}[{i - 1}]")
            out[idx] = parse_vec3(item, f"{where}[{i - 1}]")
        return out

    if isinstance(value, dict):
        for raw_key, raw_vec in value.items():
            idx = parse_index(raw_key, f"{where}.{raw_key}")
            out[idx] = parse_vec3(raw_vec, f"{where}.{raw_key}")
        return out

    raise DeclarationError(f"{where}: expected list or dict")


MAX_BASES = 3  # matches the C++ side (apps/help_proxy_sim.cpp).


def parse_bases_block(value: Any, where: str) -> list[Vec3]:
    """Parse an optional list of base-station positions (1..MAX_BASES)."""
    if value is None:
        return []
    if not isinstance(value, list):
        raise DeclarationError(f"{where}: expected a list of base positions")
    if len(value) > MAX_BASES:
        raise DeclarationError(f"{where}: at most {MAX_BASES} bases supported")
    out: list[Vec3] = []
    for i, item in enumerate(value):
        out.append(parse_vec3(item, f"{where}[{i}]"))
    return out


def parse_base_failures_block(value: Any, where: str) -> list[tuple[int, float]]:
    """Parse an optional list of {base, at_seconds} kill schedules.

    `base` may be an int (1-based slot) or a string like "base1".
    """
    if value is None:
        return []
    if not isinstance(value, list):
        raise DeclarationError(f"{where}: expected a list of base failure entries")
    out: list[tuple[int, float]] = []
    for i, item in enumerate(value):
        loc = f"{where}[{i}]"
        if not isinstance(item, dict):
            raise DeclarationError(f"{loc}: expected an object with 'base' and 'at_seconds'")
        if "base" not in item:
            raise DeclarationError(f"{loc}: missing required key 'base'")
        if "at_seconds" not in item:
            raise DeclarationError(f"{loc}: missing required key 'at_seconds'")

        raw = item["base"]
        if isinstance(raw, int):
            idx = raw
        else:
            text = str(raw).strip().lower()
            if text.startswith("base"):
                text = text[4:]
            try:
                idx = int(text)
            except ValueError:
                raise DeclarationError(f"{loc}.base: invalid value '{raw}'") from None
        if idx < 1 or idx > MAX_BASES:
            raise DeclarationError(f"{loc}.base: index {idx} out of range 1..{MAX_BASES}")

        at_s = as_float(item["at_seconds"], f"{loc}.at_seconds")
        if at_s < 0:
            raise DeclarationError(f"{loc}.at_seconds must be >= 0")
        out.append((idx, at_s))
    return out


def parse_drone_base_block(value: Any, where: str) -> dict[int, int]:
    """Parse drone -> base assignment (both 1-based).

    Accepts dicts like {"drone3": 2, "drone5": 1} or {"3": 2}.
    """
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise DeclarationError(f"{where}: expected an object mapping drone -> base index")
    out: dict[int, int] = {}
    for raw_key, raw_val in value.items():
        drone_idx = parse_drone_index(raw_key, f"{where}.{raw_key}")
        base_idx = as_int(raw_val, f"{where}.{raw_key}")
        if base_idx < 1 or base_idx > MAX_BASES:
            raise DeclarationError(
                f"{where}.{raw_key}: base index {base_idx} out of range 1..{MAX_BASES}"
            )
        out[drone_idx] = base_idx
    return out


def parse_failures_block(value: Any, where: str) -> list[tuple[int, float]]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise DeclarationError(f"{where}: expected a list of failure entries")
    out: list[tuple[int, float]] = []
    for i, item in enumerate(value):
        loc = f"{where}[{i}]"
        if not isinstance(item, dict):
            raise DeclarationError(f"{loc}: expected an object with 'drone' and 'at_seconds'")
        if "drone" not in item:
            raise DeclarationError(f"{loc}: missing required key 'drone'")
        if "at_seconds" not in item:
            raise DeclarationError(f"{loc}: missing required key 'at_seconds'")
        idx = parse_drone_index(item["drone"], f"{loc}.drone")
        at_s = as_float(item["at_seconds"], f"{loc}.at_seconds")
        if at_s < 0:
            raise DeclarationError(f"{loc}.at_seconds must be >= 0")
        out.append((idx, at_s))
    return out


def extract_base(blob: dict[str, Any], where: str) -> Vec3 | None:
    if "base_station" in blob:
        return parse_vec3(blob["base_station"], f"{where}.base_station")
    if "base" in blob:
        return parse_vec3(blob["base"], f"{where}.base")
    return None


def extract_params(blob: dict[str, Any], where: str) -> dict[str, float]:
    out: dict[str, float] = {}

    raw_params = blob.get("params")
    if raw_params is not None:
        if not isinstance(raw_params, dict):
            raise DeclarationError(f"{where}.params: expected a dictionary")
        for raw_name, raw_value in raw_params.items():
            name = normalize_param_name(str(raw_name), f"{where}.params.{raw_name}")
            out[name] = as_float(raw_value, f"{where}.params.{raw_name}")

    for raw_name, raw_value in blob.items():
        key = canonical_param_key(str(raw_name))
        if key in PARAM_KEY_ALIASES:
            name = normalize_param_name(str(raw_name), f"{where}.{raw_name}")
            out[name] = as_float(raw_value, f"{where}.{raw_name}")

    return out


def load_declaration(path: Path) -> dict[str, Any]:
    if path.suffix.lower() != ".json":
        raise DeclarationError("Only JSON declaration files are supported (.json)")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise DeclarationError("Top-level declaration must be an object/dictionary")
    return data


def resolve_simulation_entries(raw: dict[str, Any]) -> list[dict[str, Any]]:
    if "simulations" not in raw:
        raise DeclarationError("Missing required 'simulations' declaration")

    spec = raw["simulations"]

    if isinstance(spec, int):
        if spec < 1:
            raise DeclarationError("simulations count must be >= 1")
        template = raw.get("simulation_template", {}) or {}
        if not isinstance(template, dict):
            raise DeclarationError("simulation_template must be an object when simulations is a count")
        return [dict(template) for _ in range(spec)]

    if isinstance(spec, dict):
        return [spec]

    if isinstance(spec, list):
        out: list[dict[str, Any]] = []
        for i, item in enumerate(spec, start=1):
            if not isinstance(item, dict):
                raise DeclarationError(f"simulations[{i - 1}] must be an object")
            out.append(item)
        return out

    raise DeclarationError("simulations must be an integer, object, or list")


def parse_defaults(raw: dict[str, Any]) -> ScenarioDefaults:
    defaults = raw.get("defaults", {}) or {}
    if not isinstance(defaults, dict):
        raise DeclarationError("defaults must be an object")

    return ScenarioDefaults(
        base_station=extract_base(defaults, "defaults"),
        drones=parse_position_block(defaults.get("drones"), "defaults.drones", parse_drone_index),
        anchors=parse_position_block(defaults.get("anchors"), "defaults.anchors", parse_anchor_index),
        params={**DEFAULT_PARAMS, **extract_params(defaults, "defaults")},
        failures=parse_failures_block(defaults.get("failures"), "defaults.failures"),
        bases=parse_bases_block(defaults.get("bases"), "defaults.bases"),
        drone_base=parse_drone_base_block(defaults.get("drone_base"), "defaults.drone_base"),
        base_failures=parse_base_failures_block(defaults.get("base_failures"), "defaults.base_failures"),
    )


def slugify(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    return slug.strip("_")


def dedupe_scenario_names(scenarios: list[Scenario]) -> list[Scenario]:
    counts: dict[str, int] = {}
    out: list[Scenario] = []
    for scenario in scenarios:
        base = scenario.name or "sim"
        n = counts.get(base, 0) + 1
        counts[base] = n
        name = base if n == 1 else f"{base}_{n:02d}"
        out.append(
            Scenario(
                name=name,
                base_station=scenario.base_station,
                drones=scenario.drones,
                anchors=scenario.anchors,
                params=scenario.params,
                failures=scenario.failures,
                bases=scenario.bases,
                drone_base=scenario.drone_base,
                base_failures=scenario.base_failures,
            )
        )
    return out


def normalize_scenarios(raw: dict[str, Any]) -> list[Scenario]:
    scenario_defaults = parse_defaults(raw)
    entries = resolve_simulation_entries(raw)

    declared_count = raw.get("simulation_count")
    if declared_count is not None and as_int(declared_count, "simulation_count") != len(entries):
        raise DeclarationError(
            f"Declared simulation count is {declared_count}, but resolved {len(entries)} simulations"
        )

    scenarios: list[Scenario] = []
    for i, entry in enumerate(entries, start=1):
        where = f"simulations[{i - 1}]"

        if not isinstance(entry, dict):
            raise DeclarationError(f"{where} must be an object")

        base = scenario_defaults.base_station
        entry_base = extract_base(entry, where)
        if entry_base is not None:
            base = entry_base

        drones = dict(scenario_defaults.drones)
        drones.update(parse_position_block(entry.get("drones"), f"{where}.drones", parse_drone_index))

        anchors = dict(scenario_defaults.anchors)
        anchors.update(parse_position_block(entry.get("anchors"), f"{where}.anchors", parse_anchor_index))

        params = dict(scenario_defaults.params)
        params.update(extract_params(entry, where))

        failures = list(scenario_defaults.failures)
        failures.extend(parse_failures_block(entry.get("failures"), f"{where}.failures"))

        # Multi-base: scenario-level bases override defaults entirely when
        # present (merging two lists-of-positions doesn't compose sensibly).
        entry_bases = parse_bases_block(entry.get("bases"), f"{where}.bases")
        bases = entry_bases if entry_bases else list(scenario_defaults.bases)

        drone_base = dict(scenario_defaults.drone_base)
        drone_base.update(parse_drone_base_block(entry.get("drone_base"), f"{where}.drone_base"))

        base_failures = list(scenario_defaults.base_failures)
        base_failures.extend(parse_base_failures_block(entry.get("base_failures"), f"{where}.base_failures"))

        # Cross-validate: every drone_base reference must fit within configured bases.
        base_count = len(bases) if bases else 1
        for drone_idx, base_idx in drone_base.items():
            if base_idx > base_count:
                raise DeclarationError(
                    f"{where}.drone_base.drone{drone_idx}: base index {base_idx} "
                    f"exceeds configured base count {base_count}"
                )

        raw_name = entry.get("name", f"sim_{i:03d}")
        name = slugify(str(raw_name)) or f"sim_{i:03d}"

        # base_failures must reference a defined base.
        base_count = len(bases) if bases else 1
        for base_idx, _ in base_failures:
            if base_idx > base_count:
                raise DeclarationError(
                    f"{where}.base_failures: base index {base_idx} "
                    f"exceeds configured base count {base_count}"
                )

        scenarios.append(
            Scenario(
                name=name,
                base_station=base,
                drones=drones,
                anchors=anchors,
                params=params,
                failures=failures,
                bases=bases,
                drone_base=drone_base,
                base_failures=base_failures,
            )
        )

    return dedupe_scenario_names(scenarios)


def scenario_to_json_dict(scenario: Scenario) -> dict[str, Any]:
    return {
        "name": scenario.name,
        "base_station": list(scenario.base_station) if scenario.base_station is not None else None,
        "bases": [list(b) for b in scenario.bases],
        "drone_base": {f"drone{k}": v for k, v in sorted(scenario.drone_base.items())},
        "drones": {f"drone{k}": list(v) for k, v in sorted(scenario.drones.items())},
        "anchors": {f"anchor{k}": list(v) for k, v in sorted(scenario.anchors.items())},
        "params": scenario.params,
        "failures": [
            {"drone": f"drone{idx}", "at_seconds": at_s}
            for idx, at_s in sorted(scenario.failures)
        ],
        "base_failures": [
            {"base": f"base{idx}", "at_seconds": at_s}
            for idx, at_s in sorted(scenario.base_failures)
        ],
    }


# ── Scenario file writer + Docker invocation ───────────────────────────────

def write_override_file(path: Path, scenario: Scenario) -> None:
    lines: list[str] = []
    lines.append("# Generated by scripts/run_declaration.py")
    lines.append(f"# scenario={scenario.name}")

    if scenario.bases:
        # Multi-base form: emit each base explicitly.  The legacy 'base=' key
        # is dropped so the C++ parser doesn't see both forms.
        for i, (x, y, z) in enumerate(scenario.bases, start=1):
            lines.append(f"base{i}={x:.6f},{y:.6f},{z:.6f}")
    elif scenario.base_station is not None:
        x, y, z = scenario.base_station
        lines.append(f"base={x:.6f},{y:.6f},{z:.6f}")

    for idx in sorted(scenario.drones):
        x, y, z = scenario.drones[idx]
        lines.append(f"drone{idx}={x:.6f},{y:.6f},{z:.6f}")

    for idx in sorted(scenario.anchors):
        x, y, z = scenario.anchors[idx]
        lines.append(f"anchor{idx}={x:.6f},{y:.6f},{z:.6f}")

    for drone_idx in sorted(scenario.drone_base):
        lines.append(f"drone{drone_idx}_base={scenario.drone_base[drone_idx]}")

    for base_idx, at_s in sorted(scenario.base_failures):
        lines.append(f"kill_base{base_idx}={at_s:.6f}")

    for idx, at_s in sorted(scenario.failures):
        lines.append(f"kill_drone{idx}={at_s:.6f}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_docker_command(
    image: str,
    run_dir: Path,
    scenario_file: Path,
    params: dict[str, float],
    algorithm: str,
) -> list[str]:
    cmd = [
        "docker", "run", "--rm",
        "-v", f"{run_dir}:/output",
        "-v", f"{scenario_file}:/tmp/scenario.txt:ro",
        image,
        "sim",
        f"--algorithm={algorithm}",
        "--scenarioFile=/tmp/scenario.txt",
        "--animOut=/output/drone-simulation.xml",
        "--csvOut=/output/reposition.csv",
    ]
    for key, cli_name in PARAM_TO_CLI.items():
        if key in params:
            cmd.append(f"--{cli_name}={params[key]}")
    return cmd


def run_command(cmd: list[str], timeout_seconds: int, cwd: Path = REPO) -> tuple[int, str, str, bool]:
    try:
        completed = subprocess.run(
            cmd,
            cwd=cwd,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
        return (completed.returncode, completed.stdout, completed.stderr, False)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = (exc.stderr or "") + f"\nTimed out after {timeout_seconds} seconds."
        return (124, stdout, stderr, True)


# ── Metric parsing + aggregation ───────────────────────────────────────────

def parse_metrics(output: str) -> dict[str, float | int | None]:
    metrics: dict[str, float | int | None] = {}

    tri = TRI_RE.search(output)
    if tri:
        metrics["tri_min"] = float(tri.group(1))
        metrics["tri_max"] = float(tri.group(2))
        metrics["tri_avg"] = float(tri.group(3))
    else:
        metrics["tri_min"] = None
        metrics["tri_max"] = None
        metrics["tri_avg"] = None

    packet_rows = PACKET_ROW_RE.findall(output)
    if packet_rows:
        pu = pa = hp = fl = ne = ub = 0
        for row in packet_rows:
            pu += int(row[2]); pa += int(row[3]); hp += int(row[4])
            fl += int(row[5]); ne += int(row[6]); ub += int(row[7])
        metrics["rx_pos_update_total"] = pu
        metrics["rx_pos_ack_total"] = pa
        metrics["rx_help_proxy_total"] = hp
        metrics["rx_flood_total"] = fl
        metrics["rx_neighbor_total"] = ne
        metrics["rx_uwb_beacon_total"] = ub
    else:
        for k in ("rx_pos_update_total", "rx_pos_ack_total", "rx_help_proxy_total",
                  "rx_flood_total", "rx_neighbor_total", "rx_uwb_beacon_total"):
            metrics[k] = None

    dist = DISTANCE_TOTAL_RE.search(output)
    if dist:
        metrics["dist_helper_total"] = float(dist.group(1))
        metrics["dist_lost_total"] = float(dist.group(2))
        metrics["dist_all_total"] = float(dist.group(3))
    else:
        metrics["dist_helper_total"] = None
        metrics["dist_lost_total"] = None
        metrics["dist_all_total"] = None

    # ── Healing outcome / return quality / post-return stability ──
    healing = HEALING_RE.search(output)
    if healing and healing.group(2) is not None:
        metrics["healing_drones"] = int(healing.group(1))
        metrics["healing_avg_s"] = float(healing.group(2))
        metrics["healing_p50_s"] = float(healing.group(3))
        metrics["healing_p95_s"] = float(healing.group(4))
        metrics["healing_max_s"] = float(healing.group(5))
    else:
        metrics["healing_drones"] = int(healing.group(1)) if healing else None
        metrics["healing_avg_s"] = None
        metrics["healing_p50_s"] = None
        metrics["healing_p95_s"] = None
        metrics["healing_max_s"] = None

    ret_lat = RETURN_LATENCY_RE.search(output)
    if ret_lat and ret_lat.group(2) is not None:
        metrics["return_drones"] = int(ret_lat.group(1))
        metrics["return_avg_s"] = float(ret_lat.group(2))
        metrics["return_p50_s"] = float(ret_lat.group(3))
        metrics["return_p95_s"] = float(ret_lat.group(4))
        metrics["return_max_s"] = float(ret_lat.group(5))
    else:
        metrics["return_drones"] = int(ret_lat.group(1)) if ret_lat else None
        metrics["return_avg_s"] = None
        metrics["return_p50_s"] = None
        metrics["return_p95_s"] = None
        metrics["return_max_s"] = None

    recovery = RECOVERY_RE.search(output)
    if recovery:
        metrics["recovery_lost"] = int(recovery.group(1))
        metrics["recovery_armed"] = int(recovery.group(2))
        metrics["recovery_started"] = int(recovery.group(3))
        metrics["recovery_completed"] = int(recovery.group(4))
        metrics["recovery_rate_pct"] = float(recovery.group(5))
    else:
        metrics["recovery_lost"] = None
        metrics["recovery_armed"] = None
        metrics["recovery_started"] = None
        metrics["recovery_completed"] = None
        metrics["recovery_rate_pct"] = None

    rpath = RETURN_PATH_RE.search(output)
    if rpath and rpath.group(2) is not None:
        metrics["return_path_eff_avg"] = float(rpath.group(2))
        metrics["return_path_eff_p95"] = float(rpath.group(3))
        metrics["return_path_eff_max"] = float(rpath.group(4))
    else:
        metrics["return_path_eff_avg"] = None
        metrics["return_path_eff_p95"] = None
        metrics["return_path_eff_max"] = None

    boundary = BOUNDARY_RE.search(output)
    if boundary and boundary.group(2) is not None:
        metrics["boundary_avg_m"] = float(boundary.group(2))
        metrics["boundary_stddev_m"] = float(boundary.group(3))
        metrics["boundary_target_m"] = float(boundary.group(4))
    else:
        metrics["boundary_avg_m"] = None
        metrics["boundary_stddev_m"] = None
        metrics["boundary_target_m"] = None

    rearm = REARM_RE.search(output)
    if rearm:
        metrics["rearm_total"] = int(rearm.group(2))
        metrics["rearm_max"] = int(rearm.group(3))
    else:
        metrics["rearm_total"] = None
        metrics["rearm_max"] = None

    drift = STATION_DRIFT_RE.search(output)
    if drift and drift.group(2) is not None:
        metrics["station_drift_avg_m"] = float(drift.group(2))
        metrics["station_drift_p95_m"] = float(drift.group(3))
        metrics["station_drift_max_m"] = float(drift.group(4))
    else:
        metrics["station_drift_avg_m"] = None
        metrics["station_drift_p95_m"] = None
        metrics["station_drift_max_m"] = None

    coverage = FINAL_COVERAGE_RE.search(output)
    if coverage:
        metrics["final_coverage_covered"] = int(coverage.group(1))
        metrics["final_coverage_total"] = int(coverage.group(2))
        metrics["final_coverage_rate_pct"] = float(coverage.group(3))
        metrics["final_coverage_hops1"] = int(coverage.group(4))
        metrics["final_coverage_hops2"] = int(coverage.group(5))
        metrics["final_coverage_hops3plus"] = int(coverage.group(6))
        metrics["final_coverage_lost"] = int(coverage.group(7))
        metrics["final_coverage_dead"] = int(coverage.group(8)) if coverage.group(8) else 0
    else:
        metrics["final_coverage_covered"] = None
        metrics["final_coverage_total"] = None
        metrics["final_coverage_rate_pct"] = None
        metrics["final_coverage_hops1"] = None
        metrics["final_coverage_hops2"] = None
        metrics["final_coverage_hops3plus"] = None
        metrics["final_coverage_lost"] = None
        metrics["final_coverage_dead"] = None

    failures = FAILURES_RE.search(output)
    if failures:
        metrics["failures_scheduled"] = int(failures.group(1))
        metrics["failures_executed"] = int(failures.group(2))
    else:
        metrics["failures_scheduled"] = None
        metrics["failures_executed"] = None

    base_failures = BASE_FAILURES_RE.search(output)
    if base_failures:
        metrics["base_failures_scheduled"] = int(base_failures.group(1))
        metrics["base_failures_executed"] = int(base_failures.group(2))
    else:
        metrics["base_failures_scheduled"] = None
        metrics["base_failures_executed"] = None

    return metrics


def aggregate_metrics(records: list[RunRecord]) -> list[dict[str, Any]]:
    aggregates: list[dict[str, Any]] = []
    for key, label, scale in METRIC_SPECS:
        values: list[float] = []
        for record in records:
            value = record.metrics.get(key)
            if isinstance(value, (int, float)):
                values.append(float(value) * scale)

        if not values:
            aggregates.append({
                "key": key, "label": label,
                "mean": None, "stdev": None, "min": None, "max": None, "samples": 0,
            })
            continue

        stdev = 0.0 if len(values) == 1 else statistics.stdev(values)
        aggregates.append({
            "key": key, "label": label,
            "mean": statistics.mean(values),
            "stdev": stdev,
            "min": min(values),
            "max": max(values),
            "samples": len(values),
        })

    return aggregates


def print_aggregate_table(aggregates: list[dict[str, Any]], successful_runs: int) -> None:
    print("\n================ AGGREGATED METRICS ================")
    print(f"Successful runs used for averages: {successful_runs}")
    fmt = "{:<34}{:>14}{:>14}{:>14}{:>14}{:>10}"
    print(fmt.format("Metric", "mean", "stdev", "min", "max", "n"))
    print("-" * 100)
    for item in aggregates:
        if item["mean"] is None:
            print(fmt.format(item["label"], "n/a", "n/a", "n/a", "n/a", "0"))
            continue
        print(fmt.format(
            item["label"],
            f"{item['mean']:.4f}",
            f"{item['stdev']:.4f}",
            f"{item['min']:.4f}",
            f"{item['max']:.4f}",
            str(item["samples"]),
        ))


def render_markdown_summary(aggregates: list[dict[str, Any]]) -> str:
    lines: list[str] = []
    lines.append("| Metric | Mean | Std Dev | Min | Max | Samples |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for item in aggregates:
        if item["mean"] is None:
            lines.append(f"| {item['label']} | n/a | n/a | n/a | n/a | 0 |")
        else:
            lines.append(
                "| " + " | ".join([
                    item["label"],
                    f"{item['mean']:.4f}",
                    f"{item['stdev']:.4f}",
                    f"{item['min']:.4f}",
                    f"{item['max']:.4f}",
                    str(item["samples"]),
                ]) + " |"
            )
    return "\n".join(lines) + "\n"


# ── Batch runner ───────────────────────────────────────────────────────────

def run_batch(
    scenarios: list[Scenario],
    out_dir: Path,
    image: str,
    algorithm: str,
    timeout_seconds: int,
    continue_on_error: bool,
) -> list[RunRecord]:
    out_dir.mkdir(parents=True, exist_ok=True)
    runs_dir = out_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)

    (out_dir / "resolved_scenarios.json").write_text(
        json.dumps([scenario_to_json_dict(s) for s in scenarios], indent=2),
        encoding="utf-8",
    )

    records: list[RunRecord] = []
    total = len(scenarios)
    for i, scenario in enumerate(scenarios, start=1):
        run_id = f"{i:03d}_{scenario.name}"
        run_dir = runs_dir / run_id
        run_dir.mkdir(parents=True, exist_ok=True)

        scenario_file = run_dir / "scenario.txt"
        write_override_file(scenario_file, scenario)

        cmd = build_docker_command(image, run_dir, scenario_file, scenario.params, algorithm)
        (run_dir / "command.txt").write_text(
            " ".join(shlex.quote(part) for part in cmd) + "\n", encoding="utf-8"
        )

        print(f"[{i}/{total}] running {run_id}")
        start = time.perf_counter()
        rc, stdout, stderr, did_timeout = run_command(cmd, timeout_seconds)
        duration = time.perf_counter() - start

        stdout_file = run_dir / "stdout.log"
        stderr_file = run_dir / "stderr.log"
        stdout_file.write_text(stdout, encoding="utf-8")
        stderr_file.write_text(stderr, encoding="utf-8")

        metrics = parse_metrics(stdout)
        (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

        status = "ok" if rc == 0 else "failed"
        if did_timeout:
            status = "timeout"

        tri_avg = metrics.get("tri_avg")
        tri_avg_text = f"{tri_avg:.4f}m" if isinstance(tri_avg, (int, float)) else "n/a"
        helpers_started = metrics.get("helpers_started")
        helpers_total = metrics.get("helpers_total")
        print(
            f"    status={status} rc={rc} duration={duration:.2f}s "
            f"tri_avg={tri_avg_text} helpers={helpers_started}/{helpers_total}"
        )

        records.append(RunRecord(
            run_id=run_id,
            name=scenario.name,
            status=status,
            return_code=rc,
            duration_seconds=duration,
            metrics=metrics,
            params=scenario.params,
            scenario_file=str(scenario_file.relative_to(out_dir)),
            stdout_file=str(stdout_file.relative_to(out_dir)),
            stderr_file=str(stderr_file.relative_to(out_dir)),
        ))

        if rc != 0 and not continue_on_error:
            raise SystemExit(
                "A simulation run failed. Re-run with --continue-on-error to collect partial results."
            )

    return records


def default_out_dir() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return REPO / "output" / f"decl_run_{stamp}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run all simulations declared in a JSON file and report averaged metrics.",
    )
    parser.add_argument("declaration", type=Path, help="Path to the JSON declaration file")
    parser.add_argument(
        "--algorithm", choices=VALID_ALGORITHMS, default="centroid",
        help="Formation-control algorithm (default: centroid)",
    )
    parser.add_argument("--out-dir", type=Path, default=None, help="Output directory (default: output/decl_run_<timestamp>)")
    parser.add_argument("--image", default=DEFAULT_IMAGE, help=f"Docker image tag (default: {DEFAULT_IMAGE})")
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_SECONDS, help="Per-run timeout in seconds")
    parser.add_argument("--continue-on-error", action="store_true", help="Do not abort on first failed run")
    parser.add_argument("--only", action="append", default=None,
                        help="Run only scenarios whose name matches one of these values (repeatable)")
    args = parser.parse_args()

    try:
        raw = load_declaration(args.declaration)
        scenarios = normalize_scenarios(raw)
    except DeclarationError as exc:
        print(f"[ERR] declaration: {exc}")
        return 1
    except FileNotFoundError:
        print(f"[ERR] declaration file not found: {args.declaration}")
        return 1
    except json.JSONDecodeError as exc:
        print(f"[ERR] declaration is not valid JSON: {exc}")
        return 1

    if args.only:
        wanted = set(args.only)
        scenarios = [s for s in scenarios if s.name in wanted]
        missing = wanted - {s.name for s in scenarios}
        if missing:
            print(f"[ERR] --only names not found in declaration: {sorted(missing)}")
            return 1

    if not scenarios:
        print("[ERR] declaration resolved to 0 simulations")
        return 1

    out_dir = args.out_dir or default_out_dir()
    print(f"Algorithm: {args.algorithm}")
    print(f"Scenarios: {len(scenarios)}")
    print(f"Out dir:   {out_dir}")

    records = run_batch(
        scenarios=scenarios,
        out_dir=out_dir,
        image=args.image,
        algorithm=args.algorithm,
        timeout_seconds=args.timeout,
        continue_on_error=args.continue_on_error,
    )

    successful = [r for r in records if r.status == "ok"]
    aggregates = aggregate_metrics(successful)
    print_aggregate_table(aggregates, len(successful))

    summary = {
        "algorithm": args.algorithm,
        "scenario_count": len(scenarios),
        "successful_runs": len(successful),
        "failed_runs": [
            {"run_id": r.run_id, "status": r.status, "return_code": r.return_code}
            for r in records if r.status != "ok"
        ],
        "aggregates": aggregates,
        "runs": [
            {
                "run_id": r.run_id,
                "name": r.name,
                "status": r.status,
                "return_code": r.return_code,
                "duration_seconds": r.duration_seconds,
                "metrics": r.metrics,
                "params": r.params,
                "scenario_file": r.scenario_file,
                "stdout_file": r.stdout_file,
                "stderr_file": r.stderr_file,
            }
            for r in records
        ],
    }
    (out_dir / "aggregate.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (out_dir / "aggregate.md").write_text(render_markdown_summary(aggregates), encoding="utf-8")

    print(f"\nJSON:     {out_dir / 'aggregate.json'}")
    print(f"Markdown: {out_dir / 'aggregate.md'}")

    non_ok = [r for r in records if r.status != "ok"]
    return 1 if non_ok and not args.continue_on_error else (0 if not non_ok else 0)


if __name__ == "__main__":
    raise SystemExit(main())
