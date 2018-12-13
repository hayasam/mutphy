from .mutation import *
import os,shutil

def get_indices(substr,str):
    indices = [index for index, value in enumerate(str) if value ==substr]
    return indices

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
    args.append('')
    rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+','+args[1]+');\n'
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
    rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+', VALUE('+args[1]+'));\n'
    args.append(rec_line)
    return args

def parse_input_stmt(stmt,module_name,global_dic,locals_dic):
    args=[]
    input_stmt_start = stmt.index('digitalRead')
    arg_start = stmt.index('(')+1
    arg_end = stmt.rindex(')')
    args.append(stmt[arg_start:arg_end])
    if(not(args[0].isdigit())):
        pin_arg = module_name+'.'+args[0]
        args[0]=str(analyse_pin_arg(pin_arg,global_dic,local_dic))
    rec_line = stmt[0:input_stmt_start]+'VALUE('+stmt[input_stmt_start:arg_start]+'PIN'+args[0]+'));\n';
    args.append(rec_line)
    return args

def parse_pin_rep_stmt(stmt,module_name,global_dic,local_dic):
    #tone, notone, detachInterrupt
    arg_start = stmt.index('(')+1
    arg_end = stmt.rindex(')')
    arg_stmt = stmt[arg_start:arg_end]
    comma_indices = get_indices(',',arg_stmt)
    if(len(comma_indices)==0):
        comma_indices.append(arg_end-arg_start)
    args = arg_stmt.split(',')
    if(not(args[0].isdigit())):
        #pin_arg = module_name+'.'+args[0]
        #args[0]=str(analyse_pin_arg(pin_arg,global_dic,local_dic))
        pin_start = args[0].index('(')+1
        pin_end=args[0].rindex(')')
        pin_arg = args[0]
        args[0]=pin_arg[pin_start:pin_end]
        print(args[0])
    args_needed=[]
    args_needed.append(args[0])
    if(not(args[0].isdigit())):
        rec_line = stmt[0:(stmt.index('(')+1)]+'digitalPinToInterrupt(PIN'+args[0]+')'+stmt[arg_start+comma_indices[0]:]
    else:
        rec_line = stmt[0:stmt.index('(')]+'(PIN'+args[0]+stmt[arg_start+comma_indices[0]:]
    args_needed.append(rec_line)
    return args_needed

def parse_edge_detect_stmt(stmt,module_name,global_dic,local_dic): # attachInterrupt
    arg_start = stmt.index('(')+1
    arg_end = stmt.rindex(')')
    arg_stmt = stmt[arg_start:arg_end]
    comma_indices = get_indices(',',arg_stmt)
    rec_line=''
    args = arg_stmt.split(',')
    print(args);
    if(not(args[0].isdigit())):
        #pin_arg = module_name+'.'+args[0]
        #args[0]=str(analyse_pin_arg(pin_arg,global_dic,local_dic))
        pin_start = args[0].index('(')+1
        pin_end=args[0].rindex(')')
        pin_arg = args[0]
        args[0]=pin_arg[pin_start:pin_end]
        print(args[0])
    args_needed=[]
    args_needed.append(args[0])
    args_needed.append(args[2])
    rec_line = stmt[0:stmt.index('(')]+'digitalPinToInterrupt(PIN'+args[0]+')'+stmt[arg_start+comma_indices[0]:]
    args_needed.append(rec_line)
    return args_needed

def ensure_dir(file_path):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)
        
# copy other necessary files in orig_dir
def copy_other_files(orig_path,orig_file,mut_dir):
    directory = os.path.dirname(orig_path)
    all_files = os.listdir(directory)
    for file in all_files:
        if (not(orig_file+'.cpp' in file) and os.path.isfile(os.path.join(directory,file))):
            shutil.copy(os.path.join(directory,file),mut_dir)

def write_mutant(mut_dir,orig_dir,orig_file,lines,mut):
    # first copy the original dir
    shutil.copytree(orig_dir+'/original/',mut_dir)
    mut_file = mut_dir+'src/'+orig_file+".h"
    ensure_dir(mut_file)
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
    test_mut_file =dir+file_name+"_mut"+str(mid)+".cpp"
    ensure_dir(test_mut_file)
    test_f=open(test_mut_file,'w')
    current_line = 0
    for line in test_lines:
        if current_line==mark:
            line=line.replace(orig_pkg,mut_pkg)
        test_f.write(line)
        current_line+=1
    test_f.close()
