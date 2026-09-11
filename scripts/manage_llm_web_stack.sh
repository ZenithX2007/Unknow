#!/usr/bin/env bash

# Shared lifecycle helpers for the lightweight web launchers.  A previous
# launcher owns an isolated session, so its process group can be stopped
# without touching the terminal that starts the next launcher.

gen0_prepare_web_stack() {
  GEN0_WEB_STACK_PID_FILE="${GEN0_PROJECT_ROOT}/runtime_logs/llm_web_stack.pids"
  mkdir -p "${GEN0_PROJECT_ROOT}/runtime_logs"

  if [[ -f "${GEN0_WEB_STACK_PID_FILE}" ]]; then
    while IFS= read -r process_group; do
      [[ "${process_group}" =~ ^[0-9]+$ ]] || continue
      kill -TERM -- "-${process_group}" 2>/dev/null || true
    done < "${GEN0_WEB_STACK_PID_FILE}"
    rm -f "${GEN0_WEB_STACK_PID_FILE}"
    sleep 1
  fi

  # Older versions did not record the Python server PID.  Only target servers
  # serving this workspace's web_control directory, never arbitrary Python.
  local process_id command_line
  while IFS= read -r process_id; do
    [[ -r "/proc/${process_id}/cmdline" ]] || continue
    command_line="$(tr '\0' ' ' < "/proc/${process_id}/cmdline")"
    if [[ "${command_line}" == *"http.server"* &&
          "${command_line}" == *"--directory ${GEN0_PROJECT_ROOT}/web_control"* ]]; then
      kill -TERM "${process_id}" 2>/dev/null || true
    fi
  done < <(pgrep -f '[p]ython3 -m http.server' 2>/dev/null || true)
  sleep 1
}

gen0_start_managed() {
  setsid "$@" &
  local process_group=$!
  GEN0_WEB_STACK_CHILD_PIDS+=("${process_group}")
  printf '%s\n' "${process_group}" >> "${GEN0_WEB_STACK_PID_FILE}"
}

gen0_cleanup_web_stack() {
  trap - EXIT INT TERM
  local process_group
  for process_group in "${GEN0_WEB_STACK_CHILD_PIDS[@]}"; do
    kill -TERM -- "-${process_group}" 2>/dev/null || true
  done
  rm -f "${GEN0_WEB_STACK_PID_FILE}"
  wait 2>/dev/null || true
}
