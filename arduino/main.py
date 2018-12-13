from mt.mutation import *
from mt.scan import *
import sys,os,shutil

# main
orig_dir='toy_example'
orig_file='breath_lib'
orig_path=orig_dir+'/original/src/'+orig_file+'.h'
f = open(orig_path,'r')

current_line = 0
mut_map=[]
lines =[]
for line in f:
    print(line)
    lines.append(line)
    # pinMode stmt
    setup_stmt = "pinMode"
    if(setup_stmt in line):
        #print(line)
        args = parse_setup_stmt(line,orig_file,globals(),locals())
        # setup PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0]),args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # setup PIN function replacement
        mutants = apply_mutation('pin_func_replacement',get_func_rep_idx(args[1]),args[-1],current_line,args[1])
        mut_map.extend(mutants)
        # setup pull resist replacement
        if 'INPUT' in args[1]:
            mutants = apply_mutation('pull_resis_replacement',get_pull_rep_idx(args[1]),args[-1],current_line,args[1])
            mut_map.extend(mutants)
    # digitalWrite stmt
    output_stmt = "digitalWrite"
    if(output_stmt in line):
        args = parse_ouput_stmt(line,orig_file,globals(),locals())
        # output PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0]),args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # output value replacement
        mutants = apply_mutation('pin_value_replacement',0,args[-1],current_line,'VALUE')
        mut_map.extend(mutants)
        # output stmt deletion
        mutants = apply_mutation('output_stmt_deletion',0,args[-1],current_line,args[-1].strip())
        mut_map.extend(mutants)
    # digitalRead stmt
    input_stmt = "digitalRead"
    if(input_stmt in line):
        args = parse_input_stmt(line,orig_file,globals(),locals())
        # input PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0]),args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # input value replacement
        mutants = apply_mutation('pin_value_replacement',0,args[-1],current_line,'VALUE')
        mut_map.extend(mutants)
    # attachInterrupt stmt
    edge_detect_stmt = 'attachInterrupt('
    if(edge_detect_stmt in line):
        args = parse_edge_detect_stmt(line,orig_file,globals(),locals())
        # PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0]),args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # edge value replacement
        mutants = apply_mutation('edge_value_replacement',get_edge_rep_idx(args[1]),args[-1],current_line,args[1])
        mut_map.extend(mutants)
    # pin replacement statement: tone, notone, detachInterrupt
    pin_rep_stmt = ['tone','notone','detachInterrupt']
    is_pin_rep_stmt = 0
    for type in pin_rep_stmt:
        is_pin_rep_stmt = is_pin_rep_stmt or (type in line)
    if is_pin_rep_stmt:
        args = parse_pin_rep_stmt(line,orig_file,globals(),locals())
        # PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0]),args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
    current_line += 1
f.close()

# write mutants & tests into a new dir
for mut in mut_map:
    print(mut)
    mid = mut.get_mid()
    mut_dir = orig_dir+'/mut'+str(mid)+'/'
    write_mutant(mut_dir,orig_dir,orig_file,lines,mut)

    

