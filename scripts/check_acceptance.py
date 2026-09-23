#!/usr/bin/env python3
"""Evaluate one run_sim log against named acceptance criteria."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from run_metrics import analyze_log


def metric_value(metrics: dict[str, Any], path: str) -> Any:
    value: Any = metrics
    for part in path.split("."):
        value = value[part]
    return value


def sequence_contains(actual: list[Any], expected: list[Any]) -> bool:
    index = 0
    for item in actual:
        if index < len(expected) and item == expected[index]:
            index += 1
    return index == len(expected)


def evaluate_rule(metrics: dict[str, Any], rule: dict[str, Any]) -> tuple[bool, Any]:
    actual = metric_value(metrics, rule["metric"])
    expected = rule["value"]
    operation = rule["op"]
    operations = {
        "lt": lambda a, b: a < b,
        "le": lambda a, b: a <= b,
        "gt": lambda a, b: a > b,
        "ge": lambda a, b: a >= b,
        "eq": lambda a, b: a == b,
        "contains_sequence": sequence_contains,
    }
    if operation not in operations:
        raise ValueError(f"unsupported acceptance operation: {operation}")
    return bool(operations[operation](actual, expected)), actual


def evaluate_scenario(
    name: str,
    metrics: dict[str, Any],
    specification: dict[str, Any],
) -> dict[str, Any]:
    checks = []
    for rule in specification["checks"]:
        passed, actual = evaluate_rule(metrics, rule)
        checks.append({**rule, "actual": actual, "passed": passed})
    return {
        "scenario": name,
        "passed": all(item["passed"] for item in checks),
        "checks": checks,
        "metrics": metrics,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", help="run directory or log.csv")
    parser.add_argument("scenario", help="scenario key in acceptance JSON")
    parser.add_argument("--config", default="config/acceptance.json")
    parser.add_argument("--out", help="write detailed result JSON")
    args = parser.parse_args()

    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    if args.scenario not in config["scenarios"]:
        raise SystemExit(f"unknown acceptance scenario: {args.scenario}")
    source, metrics = analyze_log(args.run)
    result = evaluate_scenario(args.scenario, metrics, config["scenarios"][args.scenario])
    result["log"] = str(source.resolve())
    encoded = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.out:
        Path(args.out).write_text(encoded, encoding="utf-8")
    print(f"acceptance {args.scenario}: {'PASS' if result['passed'] else 'FAIL'}")
    for check in result["checks"]:
        mark = "PASS" if check["passed"] else "FAIL"
        print(
            f"  [{mark}] {check['metric']} {check['op']} {check['value']} "
            f"(actual={check['actual']})"
        )
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
