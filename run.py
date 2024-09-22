import argparse
from datetime import datetime
from enum import Enum
import json
import os
from queue import Queue, Empty
import subprocess
import threading
import time
import traceback
import sys
from typing import IO
import time

import engine

raw_debug_env = os.environ.get("DEBUG")
DEBUG = raw_debug_env == "1" or raw_debug_env == "true"

class RunOpponent(Enum):
    SELF = "self"
    COMPUTER_TEAM_0 = "computerTeam0"
    COMPUTER_TEAM_1 = "computerTeam1"

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = "\033[93m"
BLUE = '\033[94m'
RESET = '\033[0m'

COMMANDS_FOR_OPPONENT: dict[RunOpponent, list[tuple[str, str]]] = {
    RunOpponent.SELF: [
        ("Engine", "npm start 3001 3002", YELLOW),
        ("Team 0", "java -jar build/libs/starterpack.jar serve 3001", GREEN),
        ("Team 1", "java -jar build/libs/starterpack.jar serve 3002", BLUE),
    ],
    RunOpponent.COMPUTER_TEAM_0: [
        ("Engine", "npm start 0 9001", YELLOW),
        ("Team 1", "java -jar build/libs/starterpack.jar serve 9001", BLUE),
    ],
    RunOpponent.COMPUTER_TEAM_1: [
        ("Engine", "npm start 9001 0", YELLOW),
        ("Team 0", "java -jar build/libs/starterpack.jar serve 9001", GREEN),
    ],
}

def build_jar():
    one_worked = False
    gradle_outputs = []

    possible_gradles = [["./gradlew", "build"], ["cmd", "/c", "gradlew.bat build"]]
    for possible_gradle in possible_gradles:
        try:
            process = subprocess.Popen(
                possible_gradle,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            output, _ = process.communicate()
            exit_code = process.wait()

            gradle_outputs.append(f"{possible_gradle} exited with exit code {exit_code}")
            gradle_outputs.extend(output.splitlines())

            if exit_code == 0:
                one_worked = True
                break
        except Exception as e:
            gradle_outputs.append(str(e))

    if not one_worked:
        print("Failed to build jar:", file=sys.stderr)
        for output in gradle_outputs:
            print(output, file=sys.stderr)
        sys.exit(1)


def run(opponent: RunOpponent):
    if engine:
        engine.update_if_not_latest()

    print("Building jar...")
    build_jar()

    print(
        f"Running against opponent {opponent.value}... (might take a minute, please wait)"
    )

    info = COMMANDS_FOR_OPPONENT[opponent]
    prefixes = list(map(lambda x: x[0], info))
    commands = list(map(lambda x: x[1], info))
    colors = list(map(lambda x: x[2], info))

    now = datetime.now()
    formatted_now = now.strftime("%Y_%m_%d__%H_%M_%S")
    gamelog_name = f"log_{formatted_now}"
    output_logs_dir = f"logs/{gamelog_name}/"
    gamelog_path = os.path.join(output_logs_dir, f"gamelog.json")

    new_env = os.environ.copy()
    # Set gamelog output location, needs to be relative to engine directory
    new_env["OUTPUT"] = os.path.join("../../", gamelog_path)
    # Do not buffer output
    new_env["PYTHONUNBUFFERED"] = "1"

    # Launch each command in a separate terminal
    processes: list[subprocess.Popen] = []
    for i, command in enumerate(commands):
        process = subprocess.Popen(
            command,
            shell=True,
            cwd=f"engine/content" if i == 0 else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            text=True,
            env=new_env,
        )
        processes.append(process)

        # Start the engine first, then wait to start everything else (mac requires engine to start first)
        if i == 0:
            time.sleep(1)

    queue = Queue()

    def run_and_output(io: IO, i: int, is_err: True):
        for line in iter(io.readline, ""):
            line: str
            queue.put((is_err, time.time_ns(), i, line.strip()))

    threads: list[threading.Thread] = []
    for i in range(len(processes) - 1, -1, -1):
        process = processes[i]

        thread_stdout = threading.Thread(
            target=run_and_output, args=(process.stdout, i, False)
        )
        thread_stderr = threading.Thread(
            target=run_and_output, args=(process.stderr, i, True)
        )
        thread_stdout.start()
        thread_stderr.start()
        threads.append(thread_stdout)
        threads.append(thread_stderr)

    outputs = []

    while all(t.is_alive() for t in threads):
        try:
            item = queue.get(timeout=0.1)
            outputs.append(item)
            is_err, timestamp, i, line = item
            prefix = prefixes[i]
            color = colors[i] if not is_err else RED
            print(f"{color}[{prefix}] {line}{RESET}", flush=True, file=sys.stderr if is_err else sys.stdout)
        except Empty:
            pass

    for thread in threads:
        thread.join()

    outputs.sort(key=lambda x: x[1])

    files = []

    if not os.path.exists(output_logs_dir):
        os.makedirs(output_logs_dir, exist_ok=True)

    for i in range(len(processes)):
        filename = f"{output_logs_dir}{prefixes[i].replace(' ', '_').lower()}.txt"
        files.append(filename)
        output = list(map(lambda x: x[3], filter(lambda x: x[2] == i, outputs)))

        with open(filename, "w") as file:
            file.write("\n".join(output))

    print(
        "\nNote that output above may not be in the exact order it was output, due to terminal limitations.\n"
        + f"For separated ordered output, see: {', '.join(files)}"
    )
    print(f"For the gamelog, see: {gamelog_path}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python run.py [target]\nNote: you should not be using this directly, use the starterpack jar instead", file=sys.stderr)
        exit(2)
    target = sys.argv[1]


    for opponent in list(RunOpponent):
        if opponent.value == target:
            return run(opponent)

    print("Invalid target", file=sys.stderr)
    exit(2)


if __name__ == "__main__":
    main()
