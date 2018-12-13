from .mutation import *

def import_file(module_name,file_path):
    spec = util.spec_from_file_location(module_name, file_path)
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules[module_name] = module
    #print(globals(),locals())
    return module

def get_indices(substr,str):
    indices = [index for index, value in enumerate(str) if value ==substr]
    return indices

def parse_gpio_import_identifier(stmt):
    if "as" not in stmt:
        if "from" in stmt: # from RPi import GPIO ('from RPi import *' is not valid)
            return "GPIO"
        else: # import RPi.GPIO ('import RPi' is not valid)
            return "RPi.GPIO"
    else:
        strs = stmt.split('as')
        return strs[1]

def analyse_pin_arg(pin_arg,global_dic0,local_dic0):
    try:
        result = str(eval(pin_arg,global_dic0,local_dic0))
    except AttributeError as e:
        result = '2'
    return result
 
def parse_setup_stmt(stmt,module_name,global_dic,local_dic):
    arg_start = stmt.index('(')+1
    arg_end = stmt.rindex(')')
    arg_stmt = stmt[arg_start:arg_end]
    args = arg_stmt.split(',')
    comma_indices = get_indices(',',arg_stmt)
    if(not(args[0].isdigit())):
        arg = module_name+'.'+args[0]
        args[0]=analyse_pin_arg(arg,global_dic,local_dic)
    # without initial setup
    if len(comma_indices)==1:
        args.append('')
        rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+','+args[1]+')\n'
    # with output pin value setup
    elif "OUT" in args[1]:
        parts = args[2].split('=')
        pin_value = parts[1]
        args[2]=pin_value
        rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+','+args[1]+',initial=VALUE('+pin_value+'))\n'
    # with pull resistor setup
    else:
        rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+stmt[arg_start+comma_indices[0]:]    
    #rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+stmt[arg_start+comma_indices[0]:]
    args.append(rec_line)
    return args

def parse_ouput_stmt(stmt,module_name,global_dic,local_dic):
    arg_start = stmt.index('(')+1
    arg_end = stmt.rindex(')')
    arg_stmt = stmt[arg_start:arg_end]
    args = arg_stmt.split(',')
    if(not(args[0].isdigit())):
        pin_arg = module_name+'.'+args[0]
        args[0]=str(analyse_pin_arg(pin_arg,global_dic,local_dic))
    rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+',VALUE('+args[1]+'))\n'
    args.append(rec_line)
    return args

def parse_input_like_stmt(stmt,module_name,global_dic,local_dic):
    #input,event_detected,remove_event_detect,cleanup,add_event_callback
    args=[]
    input_stmt_start = stmt.index('GPIO.')
    input_stmt= stmt[input_stmt_start:]
    input_stmt_end = input_stmt.index(')')+1
    input_stmt = input_stmt[0:input_stmt_end]
    #print(input_stmt)
    rest_stmt_end = len(stmt)-len(input_stmt)-input_stmt_start
    rest_stmt = stmt[len(stmt)-rest_stmt_end:]
    #print(rest_stmt)
    arg_start = input_stmt.index('(')+1
    arg_end = input_stmt.rindex(')')
    arg_stmt = input_stmt[arg_start:arg_end]
    comma_indices = get_indices(',',arg_stmt)
    rec_line=''
    if(len(comma_indices)==0):
        args.append(arg_stmt)
        comma_indices.append(arg_end-arg_start)
    else:
        # only first argument is needed
        args0 = arg_stmt.split(',')
        args.append(args0[0])
    if(len(args[0])>0 and not(args[0].isdigit())):
        pin_arg = module_name+'.'+args[0]
        #print(pin_arg)
        args[0]=str(analyse_pin_arg(pin_arg,global_dic,local_dic))
    if (len(args[0])!=0) and (not args[0].isspace()):
        rec_line = stmt[0:input_stmt_start]+input_stmt[0:arg_start]+'PIN'+args[0]+input_stmt[arg_start+comma_indices[0]:]+rest_stmt
    # especially for input statement
    if '.input' in stmt:
        rec_line = rec_line[0:input_stmt_start]+'VALUE '+rec_line[input_stmt_start:]
    args.append(rec_line)
    #print(rec_line)
    return args

def parse_edge_detect_stmt(stmt,module_name,global_dic,local_dic): # wait_for_edge,add_event_detect
    arg_start = stmt.index('(')+1
    arg_end = stmt.rindex(')')
    arg_stmt = stmt[arg_start:arg_end]
    comma_indices = get_indices(',',arg_stmt)
    rec_line=''
    args = arg_stmt.split(',')
    if(not(args[0].isdigit())):
        pin_arg = module_name+'.'+args[0]
        args[0]=str(analyse_pin_arg(pin_arg,global_dic,local_dic))
    args_needed=[]
    args_needed.append(args[0])
    args_needed.append(args[1])
    rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+stmt[arg_start+comma_indices[0]:]
    args_needed.append(rec_line)
    return args_needed

def write_mutant(dir,orig_file,lines,mut):
    mut_file = dir+orig_file+"_mut"+str(mut.get_mid())+".py"
    f=open(mut_file,'w')
    mut_line_no=mut.get_line_no()
    #lines[mut_line_no]=mut.get_details()
    current_line = 0
    for line in lines:
        if(current_line==mut_line_no):
            line = mut.get_details()
        f.write(line)
        current_line += 1
    f.close()

def write_test(dir,file_name,test_lines,mark,orig_pkg,mid,mut_pkg):
    test_mut_file =dir+file_name+"_mut"+str(mid)+".py"
    test_f=open(test_mut_file,'w')
    current_line = 0
    for line in test_lines:
        if current_line==mark:
            line=line.replace(orig_pkg,mut_pkg)
        test_f.write(line)
        current_line+=1
    test_f.close()
