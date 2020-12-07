#! /usr/bin/python3
import multiprocessing
import os
import re
import subprocess
import sys
from multiprocessing.pool import ThreadPool


def call_slice(taskfile, idx, outputfile, command, timeout):
    dirs = outputfile[:outputfile.rfind("/")]
    if not os.path.exists(dirs):
        os.makedirs(dirs)
    file_handle = open(outputfile, "wb")
    p = subprocess.call(command, shell=True, stdout=file_handle, timeout=timeout*60, stderr=open(os.devnull,'w'))
    with open(taskfile, "a") as progressfile:
        progressfile.write("{} : {}\n".format(idx, p))
    print("{} finished".format(idx))


def run_pool(task_file, num_threads, timeout):
    progressfile = task_file + ".progress"
    pool = ThreadPool(num_threads)
    lines = open(task_file, "r").read().splitlines()
    output_files = lines[::2]
    commands = lines[1::2]
    for i in range(len(commands)):
        doit = True
        if os.path.exists(output_files[i]):
            content = open(output_files[i], "r").read()
            if re.findall("NO RESULT", content) or re.findall("Total", content):
                doit = False
        if doit:
            pool.apply_async(call_slice, args=(progressfile, i, output_files[i], commands[i], timeout))
    print("All tasks submitted to pool")
    pool.close()
    pool.join()


if __name__ == "__main__":
    task_file = sys.argv[1]
    run_pool(task_file, int(sys.argv[2]), int(sys.argv[3]))
