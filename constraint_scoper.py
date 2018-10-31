import cps_constraints as con

class ConstraintScope:

    def __init__(self, scopeId):
        self.scopeId = scopeId
        self.excluded_tokens = []
        self.excluded_scopes = []
        pass


    def add_excluded_tokens(self, tokens):
        for t in tokens:
            if t not in self.excluded_tokens:
                self.excluded_tokens.append(t)


class ConstraintScoper:

    def __init__(self):

        self.scope_dir = {}
        self.scope_pairs = []
        self.scoped_var_tokens = []
        self.constraint_scope_list = []
        self.current_scope = None
        pass


    def scan_cu_constraints(self):
        for var, cu_con in con.computed_unit_constraints.items():
            if len(cu_con) < 2:
                continue
            (lt, lname, units, isKnown) = cu_con[0]
            if (not con.is_only_known_unit_variable(lt.variable, lname)):
                continue
            #units = map(lambda (t,n,u,k): u, cu_con)
            #units_set = []
            #for u in units:
            #    if u not in units_set:
            #        units_set.append(u)
            #if len(units_set) < 2:
            #    continue

            for i in range(len(cu_con)-1):
                (t1, n1, u1, k1) = cu_con[i]
                (t2, n2, u2, k2) = cu_con[i+1]
                if (u1 != u2) and (t1.scopeId != t2.scopeId) and (t1.scope.type == 'If' and t2.scope.type == 'Try'):
                    # SCOPE_DIR
                    sc1 = self.scope_dir.get(t1.scopeId)
                    if not sc1:
                        self.scope_dir[t1.scopeId] = [t1]
                    else:
                        sc1.append(t1)
                    sc2 = self.scope_dir.get(t2.scopeId)
                    if not sc2:
                        self.scope_dir[t2.scopeId] = [t2]
                    else:
                        sc2.append(t2)

                    #SCOPE_PAIRS
                    if (t1.scopeId, t2.scopeId) not in self.scope_pairs:
                        self.scope_pairs.append((t1.scopeId, t2.scopeId))

                    #SCOPED_VAR_TOKENS
                    if t1 not in self.scoped_var_tokens:
                        self.scoped_var_tokens.append(t1)
                    if t2 not in self.scoped_var_tokens:
                        self.scoped_var_tokens.append(t2)


    def prepare_constraint_scopes(self):
        # TODO Upgrade it to handle various combinations of scopes
        
        n = pow(2, len(self.scope_pairs))
        for i in range(n):
            con_scope = ConstraintScope(i)
            for scp in self.scope_pairs:
                sc = scp[i]
                con_scope.add_excluded_tokens(self.scope_dir[sc])
            self.constraint_scope_list.append(con_scope)


    def check_constraint_scopes(self):
        self.scan_cu_constraints()
        self.prepare_constraint_scopes()


    def set_current_scope(self, scope):
        self.current_scope = scope


    def should_exclude_constraint(self, tokens):
        exclude = False
        for t in tokens:
            if t in self.current_scope.excluded_tokens:
                exclude = True
                break
        return exclude
            
       
    def print_constraint_scoper(self):
        print "Scope Directory: "
        print self.scope_dir
        print "Scope Pairs: "
        print self.scope_pairs 
                           

