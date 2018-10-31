import sys

this = sys.modules[__name__]

this.var_count = 0
this.variables = {}
this.non_unit_variables = []
this.int_unit_variables = []
this.multi_unit_variables = []
this.dimensionless_variables = []
this.known_unit_variables = {}

this.naming_constraints = {}
this.df_constraints = []
this.unique_df_constraints = []
this.computed_unit_constraints = {}
this.conversion_factor_constraints = []
this.unique_cf_constraints = []
this.known_symbol_constraints = {}

this.excluded_cu_constraints = []
this.derived_cu_constraints = []

this.units = []

this.variable2unitproba = {}
this.phys_corrections = {}

this.unit_prob_threshold = 0.5
this.found_ros_units = False
this.is_repeat_round = False
this.ENABLE_UNIT_LIST_FLATTENING = False
this.FOUND_DERIVED_CU_VARIABLE = False

this.DF_1 = 1
this.DF_2 = 2
this.CF_1 = 1
this.CF_2 = 2
this.CF_3 = 3


def reset_constraints():
    #this.naming_constraints = {}
    #this.df_constraints = []
    this.computed_unit_constraints = {}
    this.multi_unit_variables = []
    this.known_unit_variables = {}
    #this.conversion_factor_constraints = []
    #this.known_symbol_constraints = {}
    #this.units = []


def add_non_unit_variable(token, name):
    if (token.variable, name) not in this.non_unit_variables:
        this.non_unit_variables.append((token.variable, name))


def is_non_unit_variable(token, name):
    return ((token.variable, name) in this.non_unit_variables)


def add_int_unit_variable(token, name):
    if (token.variable, name) not in this.int_unit_variables:
        this.int_unit_variables.append((token.variable, name))


def is_int_unit_variable(token, name):
    return ((token.variable, name) in this.int_unit_variables)


def add_multi_unit_variable(root_token, token, name, units, isKnownRhs=False):
    if (root_token, token, name, units, isKnownRhs) not in this.multi_unit_variables:
        this.multi_unit_variables.append((root_token, token, name, units, isKnownRhs))


def add_dimensionless_variable(token, name):
    if (token.variable, name) not in this.dimensionless_variables:
        this.dimensionless_variables.append((token.variable, name))


def add_known_unit_variable(token, name, isKnown, isUnknown):
    knownStatus = this.known_unit_variables.get((token.variable, name))
    if not knownStatus:
        this.known_unit_variables[(token.variable, name)] = (isKnown, isUnknown)
    else:
        (k, uk) = knownStatus
        k = k or isKnown
        uk = uk or isUnknown
        this.known_unit_variables[(token.variable, name)] = (k, uk)    


def is_only_known_unit_variable(tokenvar, name):
    knownStatus = this.known_unit_variables.get((tokenvar, name))
    if knownStatus:
        (k, uk) = knownStatus
        if (k) and (not uk):
            return True
    return False


def print_known_unit_variables():
    print "Known Unit Variables:"
    for var, knownStatus in this.known_unit_variables.items():
        (k, uk) = knownStatus
        if (k) and (not uk):
            print var
            

def add_variable(token, name):
    this.var_count += 1
    this.variables[(token.variable, name)] = this.var_count
    return this.var_count 


def get_variable_id(token, name):
    return this.variables.get((token.variable, name))


def track_unit(unit):
    if unit and unit not in this.units:
        this.units.append(unit)


def add_nm_constraint(token, name, unitprobalist):
    var = this.variables.get((token.variable, name))
    if not var:    
        var = this.add_variable(token, name)
    for unitproba in unitprobalist[:3]:
        this.track_unit(unitproba[0])
    this.naming_constraints[var] = (token, name, unitprobalist)


def is_nm_constraint_present(var):
    return (var in this.naming_constraints)
 

def add_cu_constraint(ltoken, lname, units, isKnown):
    var = this.variables.get((ltoken.variable, lname))
    if not var:    
        var = this.add_variable(ltoken, lname)
    this.track_unit(units[0])
    cu_con = this.computed_unit_constraints.get(var)
    if not cu_con:
        this.computed_unit_constraints[var] = [(ltoken, lname, units, isKnown)]
    else:
        cu_con.append((ltoken, lname, units, isKnown))


def scan_and_create_cu_constraints(ltoken, lname):
    if (not this.is_only_known_unit_variable(ltoken.variable, lname)):
        return

    var = this.variables.get((ltoken.variable, lname))
    if not var:
        return

    cu_con = this.computed_unit_constraints.get(var)
    if len(cu_con) < 2:
        return
                    
    i = len(cu_con)-1
    (t1, n1, u1, k1) = cu_con[i-1]
    (t2, n2, u2, k2) = cu_con[i]
    if (u1 != u2) and (t1.scopeId != t2.scopeId) and (t1.scope.type == 'If' and t2.scope.type == 'Try'):
        u = []
        u.extend(u1)
        u.extend(u2)
        #u = [tuple(u)]
        new_con = (t2, n2, u, True)

        if (t1, n1, u1, k1) not in this.excluded_cu_constraints:
            this.excluded_cu_constraints.append((t1, n1, u1, k1))
        if (t2, n2, u2, k2) not in this.excluded_cu_constraints:
            this.excluded_cu_constraints.append((t2, n2, u2, k2))
        if new_con not in this.derived_cu_constraints:
            this.derived_cu_constraints.append(new_con)
            this.track_unit(u)
            this.ENABLE_UNIT_LIST_FLATTENING = True


def should_exclude_constraint(cu):
    return (cu in this.excluded_cu_constraints)


def add_df_constraint(ltoken, lname, rtoken, rname, df_type):
    lvar = this.variables.get((ltoken.variable, lname))
    rvar = this.variables.get((rtoken.variable, rname))
    if (not ltoken.units) and (not lvar):
        lvar = this.add_variable(ltoken, lname)
    if (not rtoken.units) and (not rvar):
        rvar = this.add_variable(rtoken, rname)
    if (ltoken.units):
        this.track_unit(ltoken.units[0])
    if (rtoken.units):
        this.track_unit(rtoken.units[0])

    if df_type == this.DF_2:
        if ((ltoken.variable, lname, rtoken.variable, rname, df_type) not in this.unique_df_constraints) and \
                ((rtoken.variable, rname, ltoken.variable, lname, df_type) not in this.unique_df_constraints):
            this.unique_df_constraints.append((ltoken.variable, lname, rtoken.variable, rname, df_type))
            this.df_constraints.append((ltoken, lname, rtoken, rname, df_type))
    else:
        this.df_constraints.append((ltoken, lname, rtoken, rname, df_type)) 


def is_df_constraint_present(token, name):
    for (lt, lname, rt, rname, df_type) in this.df_constraints:
        if (lt.Id == token.Id) and (lname == name):
            if not (rt.isKnown or this.is_only_known_unit_variable(rt.variable, rname)):
                return True
    return False


def add_cf_constraint(token, name, units, cf_type):
    var = this.variables.get((token.variable, name))
    if not var:    
        var = this.add_variable(token, name)
    this.track_unit(units[0])
    if (token.variable, name, units, cf_type) not in this.unique_cf_constraints:
        this.unique_cf_constraints.append((token.variable, name, units, cf_type))
        if this.is_repeat_round:
            this.conversion_factor_constraints.append((token, name, units, cf_type))
    if not this.is_repeat_round:
        this.conversion_factor_constraints.append((token, name, units, cf_type)) 


def add_ks_constraint(token, name, units):
    var = this.variables.get((token.variable, name))
    if not var:    
        var = this.add_variable(token, name)
    this.track_unit(units[0])
    ks_con = this.known_symbol_constraints.get(var)
    if not ks_con:
        this.known_symbol_constraints[var] = [(token, name, units)]
    else:
        ks_con.append((token, name, units))


def flatten_unit_list(units):
    temp = []
    for u in units:
        if isinstance(u, list):
            this.FOUND_DERIVED_CU_VARIABLE = True
            for e in u:
                if e not in temp:
                    temp.append(e)                
        else:
            if u not in temp:
                temp.append(u)
    return temp


