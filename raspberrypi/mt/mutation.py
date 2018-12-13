mop_rep_enum = ['pin_surround_replacement',\
                'pin_value_replacement',\
                'edge_value_replacement',\
                'pin_func_replacement',\
                'pull_resis_replacement',\
                'output_stmt_deletion',\
                'setup_removal']

pin_surround_replacement = [[3],\
                            [2,4],\
                            [3,14,15],\
                            [6,12],\
                            [5,12,13],\
                            [8,11],\
                            [7,9,11,25],\
                            [8,10,11,25],\
                            [9,24,25],\
                            [7,8,9,25],\
                            [5,6,13],\
                            [6,12,16,19],\
                            [3,4,15],\
                            [4,14,17,18],\
                            [13,19,20,26],\
                            [15,18,27],\
                            [15,17,27],\
                            [13,16,20,26],\
                            [16,19,21,26],\
                            [20,26],\
                            [23,24,17],\
                            [22,24,27],\
                            [10,22,23],\
                            [8,9,10,11],\
                            [16,19,20,21],\
                            [17,18,22]]

pin_value_replacement = [['not']]
edge_value_replacement = [['RISING','BOTH'],['FALLING','BOTH'],['RISING','FALLING']]
pin_func_replacement = [['IN)'],['OUT)']]
pull_resis_replacement = [['PUD_UP'],['PUD_DOWN']]
output_stmt_deletion = [['pass']]
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
    after_value = orig_value
    if orig_value.rfind('.') != -1:
        value_idx = orig_value.rfind('.')+1
        value = orig_value
        after_value = value[value_idx:]
    if after_value in ['True','1','HIGH']:
        value_rep_id = 1
    else:
        value_rep_id = 0
    return value_rep_id

def get_func_rep_idx(orig_value):
    if "OUT" in orig_value:
        return 0
    else:
        return 1

def get_pull_rep_idx(orig_value):
    if 'PUD_DOWN' in orig_value:
        return 0
    else:
        return 1

def get_edge_rep_idx(orig_value):
    if 'FALLING' in orig_value:
        return 0
    elif "RISING" in orig_value:
        return 1
    else:
        return 2
    
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

    
