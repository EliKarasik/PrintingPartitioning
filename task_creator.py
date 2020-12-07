#! /usr/bin/python3
import glob
import math
import os
import random

#executable = "/cs/labs/raananf/elik03/decomposition/decomposer/Slice"
executable = "/home/eli/CLionProjects/decomposer/cmake-build-release/Slice"
trainset = glob.glob("/cs/labs/raananf/elik03/decomposition/decomposer/models3d/trainset/final/*.obj")
resultset = glob.glob("/home/eli/CLionProjects/decomposer/models3d/article/*.obj")


def probability_options(shares=10):
    values = []
    for tip in range(shares+1):
        for base in range(shares-tip+1):
            for contraction in range(shares-tip-base+1):
                prandalg = (shares - tip - base - contraction) / shares
                ptip = tip / shares
                pbase = base / shares
                pcontraction = contraction / shares
                values.append([ptip, pbase, pcontraction, prandalg])
    return values


def backtrack_options(step_lin=0.005, step_exp=0.04):
    values = []
    q = 0.6
    while q < 1:
        c = 0.95
        while c < 1:
            values.append([c,q])
            c += step_lin
        q += step_exp
    return values


def two_precision(x):
    return "{:.3f}".format(x)


def create_task_file(task_name, angle, dists, backtracks, cut_limits, repeats=[0], models=trainset):
    task_file = []
    for mesh in models:
        mesh_short = mesh[mesh.rfind("/")+1:mesh.rfind(".")]
        for dist in dists:
            str_dist = list(map(two_precision, dist))
            for backtrack in backtracks:
                str_backtrack = list(map(two_precision, backtrack))
                for cut_limit in cut_limits:
                    for repeat in repeats:
                        output_file = "{}/cut{}/backtrack{}/dist{}/{}_{}.txt".format(task_name,
                            cut_limit, "_".join(str_backtrack), "_".join(str_dist), mesh_short, repeat)
                        command = "{} --platform offscreen --time=30 --silent -a --methods={} --bt-linear={} --bt-exp={} --cut-limit={} --angle={} {}".format(
                            executable, ",".join(str_dist), backtrack[0], backtrack[1], cut_limit, angle, mesh)
                        task_file += [output_file + "\n" + command]
    return task_file


def split_task_file(task_name, task_file, jobs, offset=0):
    random.shuffle(task_file)
    per_task = math.ceil(len(task_file) / jobs)
    if not os.path.exists("learning/"+task_name):
        os.makedirs("learning/"+task_name)
    for x in range(jobs):
        with open("learning/{0}/{0}{1}".format(task_name, x+offset), "w") as outputfile:
            outputfile.write("\n".join(task_file[x*per_task:(x+1)*per_task]))

split_task_file("resultnarrow", create_task_file("resultnarrow", 135, [[0.5,0.3,0.1,0.1]], [(0.985, 0.76)], [3000], repeats=range(3), models=resultset), 1)

split_task_file("resultwide", create_task_file("resultwide", 150, [[0.5,0.3,0.1,0.1]], [(0.965, 0.72)], [1000], repeats=range(3), models=resultset), 1)

#split_task_file("new2btnarrow", create_task_file("new2btnarrow", 135, [[0.5,0.3,0.1,0.1]], backtrack_options(), [1000,3000,5000], repeats=range(3)), 400)

#split_task_file("new2btnarrow", create_task_file("new2btnarrow", 135, [[0.5,0.3,0.1,0.1]], backtrack_options(0.005/3, 0.04/3), [1000], repeats=range(3,10)), 400)

#split_task_file("new2btwide", create_task_file("new2btwide", 150, [[0.5,0.3,0.1,0.1]], backtrack_options(), [1000,3000,5000], repeats=range(3)), 400)

#split_task_file("hugebtnarrow", create_task_file("hugebtnarrow", 135, [[0.5,0.3,0.1,0.1]], backtrack_options(0.02 / 3), [1000], repeats=range(5)), 400)

#split_task_file("newbtwide", create_task_file("newbtwide", 150, [[0.5,0.3,0.1,0.1]], backtrack_options(0.02), [1000,3000,5000], repeats=range(3)), 400)

#huge_bt = create_task_file("hugebt", 135, [[0.5,0.3,0.1,0.1]], backtrack_options(0.01 * (2 / 3)), [1000], repeats=range(3))
#split_task_file("huge_bt", huge_bt, 800)

#split_task_file("bt5000wider", create_task_file("bt5000wider", 150, [[0.5,0.3,0.1,0.1]], backtrack_options(0.02), [5000], repeats=range(3)), 400)

#split_task_file("prob3000wider", create_task_file("prob3000wider", 150, probability_options(), [(0.98, 0.82)], [3000], repeats=range(3)), 400)

#split_task_file("bt3000narrow", create_task_file("bt3000narrow", 135, [[0.5,0.3,0.1,0.1]], backtrack_options(0.02), [3000], repeats=range(3)), 400)

#problearn = create_task_file("problearn", 135, probability_options(), backtracks=[(0.96, 0.84)], cut_limits=[5000])
#split_task_file("problearn", problearn, 6)

#newbt5000learn = create_task_file("newbt5000learn", 135, [[0.5,0.3,0.1,0.1]], backtracks=backtrack_options(), cut_limits=[5000], repeats=[0,1])
#split_task_file("newbt5000learn", newbt5000learn, 400)

#prob5000learn = create_task_file("prob5000learn", 135, probability_options(), backtracks=[(0.90, 0.88)], cut_limits=[5000], repeats=[1,2])
#split_task_file("prob5000learn", prob5000learn, 800)

#bt5000learn150 = create_task_file("angle150_bt5000learn", 150, [[0.5,0.3,0.1,0.1]], #backtracks=backtrack_options(), cut_limits=[5000], repeats=[0,1,2])
#split_task_file("angle150_bt5000learn", bt5000learn150, 400)



