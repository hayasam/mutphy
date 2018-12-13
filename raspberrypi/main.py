from mt.mutation import *
from mt.scan import *
import sys,os

# main
#orig_dir='example'
#orig_file='basic'
orig_dir='kompline_lorry'
orig_file = 'chassis'
f = open(orig_dir+'/'+orig_file+".py",'r')
# import file
#from example import basic
from kompline_lorry import chassis

current_line = 0
mut_map=[]
lines =[]
for line in f:
    #print(line)
    lines.append(line)
    # import stmt
    #if(("import" in line) and ("RPi" in line)):
    #    gpio_identifier = parse_gpio_import_identifier(line)
    gpio_identifier='GPIO'
    # setup stmt
    setup_stmt = gpio_identifier+".setup"
    if(setup_stmt in line):
        #print(line)
        args = parse_setup_stmt(line,orig_file,globals(),locals())
        # setup PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0])-2,args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # setup PIN function replacement
        before = args[1]
        replacement_part_start = args[1].rfind('.')+1
        rec_line = args[-1]
        if 'OUT' in args[1]:
            func_idx = rec_line.rfind('OUT')+3
        else:
            func_idx = rec_line.rfind('IN')+2
        #print(line[func_idx:])
        replacement_part = before[replacement_part_start:]+rec_line[func_idx:-1]
        #print(replacement_part)
        mutants = apply_mutation('pin_func_replacement',get_func_rep_idx(args[1]),args[-1],current_line,replacement_part)
        mut_map.extend(mutants)
        # setup inital value replacement
        if(len(args[2])!=0):
            # output value replacement
            if 'OUT' in args[1]:
                mutants = apply_mutation('pin_value_replacement',0,args[-1],current_line,'VALUE')
                mut_map.extend(mutants)
            # input pull resistor replacement
            if 'IN' in args[1]:
                before = args[2]
                replacement_part_start = args[2].rfind('.')+1
                replacement_part = before[replacement_part_start:] 
                mutants = apply_mutation('pull_resis_replacement',get_pull_rep_idx(args[2]),args[-1],current_line,replacement_part)
                mut_map.extend(mutants)
            # setup initial removal
            before = args[-1]
            replacement_part_start = args[-1].rfind(',')
            replacement_part = before[replacement_part_start:-2]
            mutants = apply_mutation('setup_removal',0,args[-1],current_line,replacement_part)
            mut_map.extend(mutants)
    # output stmt
    output_stmt = gpio_identifier+".output"
    if(output_stmt in line):
        args = parse_ouput_stmt(line,orig_file,globals(),locals())
        # output PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0])-2,args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # output value replacement
        #value_rep_idx = get_value_rep_idx(args[1])
        mutants = apply_mutation('pin_value_replacement',0,args[-1],current_line,'VALUE')
        mut_map.extend(mutants)
        # output stmt deletion
        mutants = apply_mutation('output_stmt_deletion',0,args[-1],current_line,args[-1].strip())
        mut_map.extend(mutants)
    # input-like stmt: input,event_detected,remove_event_detect,cleanup, add_event_callback,PWM
    input_like_stmt = [gpio_identifier+'.input',gpio_identifier+'.event_detected',gpio_identifier+'.remove_event_detect',gpio_identifier+'.cleanup',gpio_identifier+'.add_event_callback',gpio_identifier+'.PWM']
    is_input_like_stmt = 0
    for type in input_like_stmt:
        is_input_like_stmt = is_input_like_stmt or (type in line)
    if is_input_like_stmt:
        args = parse_input_like_stmt(line,orig_file,globals(),locals())
        # PIN replacement
        if len(args[0]) != 0:
            mutants = apply_mutation('pin_surround_replacement',int(args[0])-2,args[-1],current_line,'PIN'+args[0])
            mut_map.extend(mutants)
        # input value replacement
        if '.input' in line:
            mutants = apply_mutation('pin_value_replacement',0,args[-1],current_line,'VALUE')
            mut_map.extend(mutants)
    # edge detect stmt: wait_for_edge,add_event_detect
    edge_detect_stmt = [gpio_identifier+'.wait_for_edge',gpio_identifier+'.add_event_detect']
    is_edge_detect_stmt = 0
    for type in edge_detect_stmt:
        is_edge_detect_stmt = is_edge_detect_stmt or (type in line)
    if is_edge_detect_stmt:
        args = parse_edge_detect_stmt(line,orig_file,globals(),locals())
        # PIN replacement
        mutants = apply_mutation('pin_surround_replacement',int(args[0])-2,args[-1],current_line,'PIN'+args[0])
        mut_map.extend(mutants)
        # edge value replacement
        before = args[1]
        replacement_part_start = args[1].rfind('.')+1
        replacement_part = before[replacement_part_start:] 
        mutants = apply_mutation('edge_value_replacement',get_edge_rep_idx(args[1]),args[-1],current_line,replacement_part)
        mut_map.extend(mutants)
    current_line += 1
f.close()
            
# scan tests
test_file_name='test_'+orig_file
test_f = open(orig_dir+'/'+test_file_name+'.py','r')
test_lines=[]
current_line = 0
for line in test_f:
    test_lines.append(line)
    if(('import' in line) and (orig_file in line)):
        mark=current_line
        print(mark)
    current_line += 1
test_f.close()

# write mutants & tests into files
mut_dir = orig_dir+'/mut/'
for mut in mut_map:
    print(mut)
    write_mutant(mut_dir,orig_file,lines,mut)
    mid = mut.get_mid()
    mut_pkg = orig_file+'_mut'+str(mid)
    write_test(mut_dir,test_file_name,test_lines,mark,orig_file,mid,mut_pkg)

