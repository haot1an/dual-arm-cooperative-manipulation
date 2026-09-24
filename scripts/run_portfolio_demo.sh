#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${repo_root}/build"
record=false
run_tests=true

usage() {
  printf '%s\n' \
    "Usage: ./scripts/run_portfolio_demo.sh [--record] [--skip-tests]" \
    "" \
    "  --record       record baseline.mp4 and autonomous_qp.mp4" \
    "  --skip-tests   skip the full CTest suite"
}

while (($#)); do
  case "$1" in
    --record) record=true ;;
    --skip-tests) run_tests=false ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

stamp="$(date +%Y%m%d_%H%M%S)"
artifact_dir="${repo_root}/artifacts/portfolio_demo/${stamp}"
mkdir -p "${artifact_dir}/plots"

printf '[portfolio] configure and build\n'
cmake -S "${repo_root}" -B "${build_dir}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}" -j"$(nproc)"

if ${run_tests}; then
  printf '[portfolio] run full test suite\n'
  ctest --test-dir "${build_dir}" --output-on-failure \
    | tee "${artifact_dir}/ctest.txt"
fi

common_args=(
  --scene slot_avoid
  --headless
  --duration 20
  --camera cam_front
  --set simulation.contacts=false
  --set 'disturbances=[]'
  # 本作品集 A/B 展示的是状态机 governor（slot_avoid 默认已改为 CBF，见 docs/cbf_reference_governor.md）
  --set controller.torque_qp.reference_governor.mode=state_machine
)

baseline_record=()
autonomous_record=()
if ${record}; then
  baseline_record=(--record "${artifact_dir}/baseline.mp4")
  autonomous_record=(--record "${artifact_dir}/autonomous_qp.mp4")
fi

printf '[portfolio] run straight cooperative baseline\n'
"${build_dir}/run_sim" \
  "${common_args[@]}" \
  --controller coop \
  "${baseline_record[@]}" \
  | tee "${artifact_dir}/baseline_console.txt"

printf '[portfolio] run autonomous QP controller\n'
"${build_dir}/run_sim" \
  "${common_args[@]}" \
  --controller qp_coop \
  "${autonomous_record[@]}" \
  | tee "${artifact_dir}/autonomous_console.txt"

baseline_run="$(sed -n 's/^\[run_sim\] log: //p' "${artifact_dir}/baseline_console.txt" | tail -n 1)"
autonomous_run="$(sed -n 's/^\[run_sim\] log: //p' "${artifact_dir}/autonomous_console.txt" | tail -n 1)"
if [[ -z "${baseline_run}" || -z "${autonomous_run}" ]]; then
  printf '[portfolio] failed to locate generated log directories\n' >&2
  exit 1
fi

printf '[portfolio] generate comparison plots\n'
python3 "${repo_root}/scripts/plot_log.py" \
  "${baseline_run}" "${autonomous_run}" \
  --labels straight_coop autonomous_qp \
  --out "${artifact_dir}/plots"

printf '[portfolio] compute acceptance metrics\n'
python3 "${repo_root}/scripts/summarize_portfolio_demo.py" \
  "${baseline_run}" "${autonomous_run}" \
  --out "${artifact_dir}"

printf '[portfolio] complete: %s\n' "${artifact_dir}"
