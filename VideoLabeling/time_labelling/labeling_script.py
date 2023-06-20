import json
import os
import copy

os.chdir(os.getcwd()+'/VideoLabeling')
with open('improvisation4.json') as improv:
    raw_sequence = json.load(improv)
    with open('temp.json') as labels:
        labels = json.load(labels)
        if not len(labels)==len(raw_sequence):
            print('length error!')
        labelled_sequence = copy.deepcopy(raw_sequence)
        labelled_sequence[0]['start_time'] = 0
        for i in range(len(raw_sequence)-1):
            labelled_sequence[i]['end_time'] = labels[i]
            labelled_sequence[i+1]['start_time'] = labels[i]
        labelled_sequence[-1]['end_time'] = labels[-1]

with open('impro4_labelled.json', 'w') as f:
    json.dump(labelled_sequence, f)
