from pgm.pgmplayer import PGMPlayer
import cps_constraints as con
from operator import itemgetter
import uuid
import os

class ConstraintSolver:

    def __init__(self, my_con_collector, my_con_scoper, SHOULD_USE_CONSTRAINT_SCOPING=False):
        self.con_collector = my_con_collector
        self.con_scoper = my_con_scoper
        self.SHOULD_PRINT_VARIABLE_TYPES = False
        self.SHOULD_USE_CONSTRAINT_SCOPING = SHOULD_USE_CONSTRAINT_SCOPING
        self.ENABLE_SCOPER = False
        self.pred2pgmvar = {}
        self.pgmvar2pred = {}
        self.uuid = str(uuid.uuid4())


    def solve(self):
        #print con.units
        #print con.non_unit_variables
        #print "Dimensionless:"
        #print con.dimensionless_variables
        
        self.pred2pgmvar = {}
        self.pgmvar2pred = {}
        var2unitproba = {}

        for unit in con.units:
            fg_filename = "pgm/predict_" + str(unit).replace(" ", "") + self.uuid + ".fg"
            player = self.prepare(fg_filename, unit)
            pgmvar2proba = player.compute_marginals()
            #print {v.name: '%.4f' % (1.0 - p) for v, p in pgmvar2proba.iteritems()}
            os.remove(fg_filename)

            for pred, pgmvar in self.pred2pgmvar.iteritems():
                self.pgmvar2pred[pgmvar] = pred

            #print '---------------------'
            #print 'Probabilistic Units:'
            #print '---------------------'
   
            for v, p in pgmvar2proba.iteritems():
                if v.name in self.pgmvar2pred:
                    (token, name, u) = self.pgmvar2pred[v.name]
                    #print '%s: %s = %s = %.4f' % (v.name, name, unit, 1.0-p)

                    if (token, name) in var2unitproba:
                        var2unitproba[(token, name)].append((unit, 1.0-p))
                    else:
                        var2unitproba[(token, name)] = [(unit, 1.0-p)]

            #print '---------------------' + '\n'

        for v in var2unitproba:
            var2unitproba[v].sort(key=itemgetter(1), reverse=True)
            if (var2unitproba[v][0][1] == 0.5):
                continue
            if self.SHOULD_PRINT_VARIABLE_TYPES:
                print '%s:\n%s\n' % (v[1], var2unitproba[v])

        con.variable2unitproba = var2unitproba
        #con.reset_constraints()        

        return var2unitproba
              
   
    def prepare(self, fg_filename, unit):
        if self.SHOULD_USE_CONSTRAINT_SCOPING and self.con_scoper.constraint_scope_list:
            self.ENABLE_SCOPER = True

        player = PGMPlayer(fg_filename)

        self.process_nm_constraints(player, unit)
        self.process_cu_constraints(player, unit)
        self.process_df_constraints(player, unit)
        self.process_cf_constraints(player, unit)
        self.process_ks_constraints(player, unit)

        return player
        

    def process_nm_constraints(self, pgm_player, unit):
        for var, nm_con in con.naming_constraints.items():
            (lt, lname, unitprobalist) = nm_con
            var = con.variables.get((lt.variable, lname))
            if var:
                nv = 'n'+ str(var)
                pv = 'p'+ str(var)
                p = 0.0
                for (un, pr) in unitprobalist:
                    if (un == unit):
                        p = pr
                        break
                    
                pgm_player.add_factor(left=[], right=[nv],
                                      states=[0, 1],
                                      proba=p,
                                      comment=nv + ' = 1')
                pgm_player.add_factor(left=[nv], right=[pv],
                                      states=[1, 0, 1, 1],
                                      proba=0.7,
                                      comment=nv + ' -> ' + pv)
                #print nv + ': (' + lname + ', ' + str(unit) + ', ' + str(p) + ')'
                #print nv + ' -> ' + pv

                if (lt.variable, lname, str(unit)) not in self.pred2pgmvar:
                    self.pred2pgmvar[(lt.variable, lname, str(unit))] = pv
                    

    def process_cu_constraints(self, pgm_player, unit):
        for var, cu_con in con.computed_unit_constraints.items():
            (lt, lname, units, isKnown) = cu_con[0]
            var = con.variables.get((lt.variable, lname))
            if var:
                cv = 'c'+ str(var)
                pv = 'p'+ str(var)
                p = 0.0
                p_fwd = 0.95 if con.found_ros_units else 0.7
                no_factor = False
                for (t, n, un, isKnown) in cu_con:
                    if self.ENABLE_SCOPER and self.con_scoper.should_exclude_constraint([t]):
                        continue
                    if con.should_exclude_constraint((t, n, un, isKnown)):
                        no_factor = True
                        continue

                    if (unit in un):
                        p = 1.0 if isKnown else 0.8
                        if isKnown:
                            break

                if no_factor and p == 0.0:
                    continue

                pgm_player.add_factor(left=[], right=[cv],
                                      states=[0, 1],
                                      proba=p,
                                      comment=cv + ' = 1')
                pgm_player.add_factor(left=[cv], right=[pv],
                                      states=[1, 0, 1, 1],
                                      proba=p_fwd,
                                      comment=cv + ' -> ' + pv)
                #print cv + ' = 1: (' + lname + ', ' + str(unit) + ', ' + str(p) + ')'
                #print cv + ' -> ' + pv

                if (lt.variable, lname, str(unit)) not in self.pred2pgmvar:
                    self.pred2pgmvar[(lt.variable, lname, str(unit))] = pv

        for (lt, lname, un, isKnown) in con.derived_cu_constraints:
            var = con.variables.get((lt.variable, lname))
            if var:
                cv = 'c'+ str(var)
                pv = 'p'+ str(var)
                p = 0.0
                p_fwd = 0.95 if con.found_ros_units else 0.7
                
                if (unit == un):
                    p = 1.0 if isKnown else 0.8

                pgm_player.add_factor(left=[], right=[cv],
                                      states=[0, 1],
                                      proba=p,
                                      comment=cv + ' = 1')
                pgm_player.add_factor(left=[cv], right=[pv],
                                      states=[1, 0, 1, 1],
                                      proba=p_fwd,
                                      comment=cv + ' -> ' + pv)
                #print cv + ' = 1: (' + lname + ', ' + str(unit) + ', ' + str(p) + ')'
                #print cv + ' -> ' + pv

                if (lt.variable, lname, str(unit)) not in self.pred2pgmvar:
                    self.pred2pgmvar[(lt.variable, lname, str(unit))] = pv


    def process_df_constraints(self, pgm_player, unit):
        for (lt, lname, rt, rname, df_type) in con.df_constraints:
            if self.ENABLE_SCOPER and self.con_scoper.should_exclude_constraint([lt, rt]):
                continue

            var1 = con.variables.get((lt.variable, lname))
            var2 = con.variables.get((rt.variable, rname))
            if var1 and var2 and (var1 != var2):
                pv1 = 'p'+ str(var1)
                pv2 = 'p'+ str(var2)
                pgm_player.add_factor(left=[pv1], right=[pv2],
                                      states=[1, 0, 1, 1],
                                      proba=0.95,
                                      comment=pv1 + ' -> ' + pv2)
                pgm_player.add_factor(left=[pv2], right=[pv1],
                                      states=[1, 0, 1, 1], 
                                      proba=0.95,
                                      comment=pv2 + ' -> ' + pv1)
                #print pv1 + ' -> ' + pv2
                #print pv2 + ' -> ' + pv1

                if (lt.variable, lname, str(unit)) not in self.pred2pgmvar:
                    self.pred2pgmvar[(lt.variable, lname, str(unit))] = pv1
                if (rt.variable, rname, str(unit)) not in self.pred2pgmvar:
                    self.pred2pgmvar[(rt.variable, rname, str(unit))] = pv2               

            else:
                if lt.isKnown and (not rt.isKnown):
                    dv2 = 'd'+ str(var2)
                    pv2 = 'p'+ str(var2)
                    p = 0.95 if (lt.units[0] == unit) else 0.0
                    pgm_player.add_factor(left=[], right=[dv2],
                                          states=[0, 1],
                                          proba=p,
                                          comment=dv2 + ' = 1')
                    pgm_player.add_factor(left=[dv2], right=[pv2],
                                          states=[1, 0, 1, 1], 
                                          proba=0.95,
                                          comment=dv2 + ' -> ' + pv2)
                    #print dv2 + ' = 1: (' + rname + ', ' + str(unit) + ', ' + str(p) + ')'
                    #print dv2 + ' -> ' + pv2
                    
                    if (rt.variable, rname, str(unit)) not in self.pred2pgmvar:
                        self.pred2pgmvar[(rt.variable, rname, str(unit))] = pv2
                elif rt.isKnown and (not lt.isKnown):
                    dv1 = 'd'+ str(var1)
                    pv1 = 'p'+ str(var1)
                    p = 0.95 if (rt.units[0] == unit) else 0.0
                    pgm_player.add_factor(left=[], right=[dv1],
                                          states=[0, 1],
                                          proba=p,
                                          comment=dv1 + ' = 1')
                    pgm_player.add_factor(left=[dv1], right=[pv1],
                                          states=[1, 0, 1, 1], 
                                          proba=0.95,
                                          comment=dv1 + ' -> ' + pv1)
                    #print dv1 + ' = 1: (' + lname + ', ' + str(unit) + ', ' + str(p) + ')'
                    #print dv1 + ' -> ' + pv1
                    
                    if (lt.variable, lname, str(unit)) not in self.pred2pgmvar:
                        self.pred2pgmvar[(lt.variable, lname, str(unit))] = pv1

         
    def process_cf_constraints(self, pgm_player, unit):
        for (t, name, units, cf_type) in con.conversion_factor_constraints:
            var = con.variables.get((t.variable, name))
            if var:
                fv = 'f'+ str(var)
                pv = 'p'+ str(var)
                p = 0.0
                if (units[0] == unit):
                    p = 0.95 if (cf_type == con.CF_3) else 0.9 

                pgm_player.add_factor(left=[], right=[fv],
                                      states=[0, 1],
                                      proba=p,
                                      comment=fv + ' = 1')
                pgm_player.add_factor(left=[fv], right=[pv],
                                      states=[1, 0, 1, 1],
                                      proba=0.95,
                                      comment=fv + ' -> ' + pv)
                #print fv + ' = 1: (' + name + ', ' + str(unit) + ', ' + str(p) + ')'
                #print fv + ' -> ' + pv

                if (t.variable, name, str(unit)) not in self.pred2pgmvar:
                    self.pred2pgmvar[(t.variable, name, str(unit))] = pv


    def process_ks_constraints(self, pgm_player, unit):
        for var, ks_con in con.known_symbol_constraints.items():
            (token, name, units) = ks_con[0]
            var = con.variables.get((token.variable, name))
            if var:
                kv = 'k'+ str(var)
                pv = 'p'+ str(var)
                p = 0.0
                for (t, n, un) in ks_con:
                    p = 0.0
                    if (un[0] == unit):
                        p = 0.95

                    pgm_player.add_factor(left=[], right=[kv],
                                          states=[0, 1],
                                          proba=p,
                                          comment=kv + ' = 1')
                    pgm_player.add_factor(left=[kv], right=[pv],
                                          states=[1, 0, 1, 1],
                                          proba=0.95,
                                          comment=kv + ' -> ' + pv)
                    #print kv + ' = 1: (' + name + ', ' + str(unit) + ', ' + str(p) + ')'
                    #print kv + ' -> ' + pv

                    if (token.variable, name, str(unit)) not in self.pred2pgmvar:
                        self.pred2pgmvar[(token.variable, name, str(unit))] = pv


