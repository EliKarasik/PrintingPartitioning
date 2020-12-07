import glob
import re

def single(l):
    return int(l[0])

def process_file(fname):
    try:
        content = open(fname, "r").read()
        fparts = fname.split("/")
        limit = fparts[1]
        backtrack = fparts[2][len("backtrack")+1:]
        dist = fparts[3][len("dist")+1:]
        model_full = fparts[4]
        last_under = model_full.rfind("_")
        model = model_full[:last_under]
        attempt = model_full[last_under+1:model_full.rfind(".")]
        if re.findall("NO RESULT", content):
            total_cuts = -1
            total_time = -1
            final_shapes = 50
        else:
            total_time = single(re.findall("Total time: ([0-9]*)", content))
            total_cuts = single(re.findall("Total Cuts done: ([0-9]*)", content))
            final_shapes = single(re.findall("Shapes after merge: ([0-9]*)", content))
        return {
            "limit": limit,
            "backtrack": backtrack,
            "dist": dist,
            "model": model,
            "attempt": attempt,
            "time": total_time,
            "cuts": total_cuts,
            "parts": final_shapes
        }
    except:
        return ""

def process_all():
    all_files = glob.glob("btlearn/*/*/*/*")
    print("reading {0} files".format(len(all_files)))
    all_data = []
    for i in range(len(all_files)):
        if i % 100 == 0:
            print(i)
        all_data.append(process_file(all_files[i]))
    filtered_data = list(filter(lambda x: x != "", all_data))
    print(len(all_data) - len(filtered_data), " crashes")
    return filtered_data
    
def by_field(data, field='backtrack'):
    uniques = set(map(lambda x: x[field], data))
    grouped = {}
    scores = []
    
    for u in uniques:
        s = 0
        relevant = list(filter(lambda x: x[field] == u, data))
        for res in relevant:
            s += res['parts']
        grouped[u] = relevant
        scores.append((u,len(relevant),s))
    scores = sorted(scores, key=lambda x: x[2])
    for r in scores:
        print(r)
    return grouped
    

