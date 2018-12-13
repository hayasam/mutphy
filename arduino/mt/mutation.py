mop_rep_enum = ['pin_surround_replacement',\
                'pin_value_replacement',\
                'edge_value_replacement',\
                'pin_func_replacement',\
                'pull_resis_replacement',\
                'output_stmt_deletion',\
                'setup_removal']

pin_surround_replacement = [[1],\
                            [0,2],\
                            [1,3],\
                            [2,4],\
                            [3,5],\
                            [4,6],\
                            [5,7],\
                            [6,8],\
                            [7,9],\
                            [8,10],\
                            [9,11],\
                            [10,12],\
                            [11,13],\
                            [12]]

pin_value_replacement = [['!']]
edge_value_replacement = [['CHANGE','RISING','FALLING'],['LOW','RISING','FALLING'],['LOW','CHANGE','FALLING'],['LOW','CHANGE','RISING']]
pin_func_replacement = [['INPUT'],['OUTPUT']]
pull_resis_replacement = [['INPUT_PULLUP'],['INPUT']]
output_stmt_deletion = [['']]
setup_removal = [['']]

mop_rep_list = []
mop_rep_list.append(pin_surround_replacement)
mop_rep_list.append(pin_value_replacement)
mop_rep_list.append(edge_value_replacement)
mop_rep_list.append(pin_func_replacement)
mop_rep_list.append(pull_resis_replacement)
mop_rep_list.append(output_stmt_deletion)
mop_rep_list.append(setup_removal)

def apply_mutation(mop,orig,line,lno,orig_with_mark):
    mutants = []
    mop_id=mop_rep_enum.index(mop)
    mop_replacement = mop_rep_list[mop_id]
    replacements = mop_replacement[orig]
    for rep in replacements:
        mut_line = line.replace(orig_with_mark,str(rep))
        # remove additional marks
        mut_line = mut_line.replace('PIN','')
        mut_line = mut_line.replace('VALUE','')
        mutant = Mutant(lno,mop,mut_line)
        mutants.append(mutant)
    return mutants

def get_value_rep_idx_by_execution(orig_value):
    if eval(orig_value) == 1:
        return 0
    else:
        return 1
    
def get_value_rep_idx(orig_value):
    if 'LOW' in orig_value:
        value_rep_id = 0
    else:
        value_rep_id = 1
    return value_rep_id

def get_func_rep_idx(orig_value):
    if "OUTPUT" in orig_value:
        return 0
    else:
        return 1

def get_pull_rep_idx(orig_value):
    if 'INPUT' in orig_value:
        return 0
    else:
        return 1

def get_edge_rep_idx(orig_value):
    if 'LOW' in orig_value:
        return 0
    elif 'CHANGE' in orig_value:
        return 1
    elif 'RISING' in orig_value:
        return 2
    else:
        return 3
      
    
class Mutant:
    mut_size = 0
    
    def __init__(self,line_no,mop,details):
        self.mid=Mutant.mut_size
        Mutant.mut_size += 1
        self.line_no = line_no
        self.mop=mop
        self.details=details

    def __str__(self):
        return "Mutation "+str(self.mid)+":"+self.mop+":"+self.details.strip()+" in Line "+str(self.line_no)

    def get_mid(self):
        return self.mid

    def get_line_no(self):
        return self.line_no

    def get_mop(self):
        return self.mop

    def get_details(self):
        return self.details

    
