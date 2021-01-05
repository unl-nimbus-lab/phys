#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


import cppcheckdata                 # http://cppcheck.sourceforge.net/
from tree_walker import TreeWalker
import cps_constraints as con
import networkx as nx
import os
from collections import OrderedDict


class ConstraintCollector:

    def __init__(self, my_type_miner):
        self.SHOULD_PRINT_CONSTRAINTS = False
        self.type_miner = my_type_miner
        self.current_file_under_analysis = ''
        self.source_file = ''
        self.source_file_lines = []
        self.function_graph = nx.DiGraph()
        self.all_function_graphs = []
        self.all_sorted_analysis_unit_dicts = []
        self.should_sort_by_function_graph = True
        self.should_abandon_early = True
        self.configurations = []
        self.vnh = None


    def init_cppcheck_config_data_structures(self, cppcheck_configuration):  
        '''  AUGMENT SCOPE DATA STRUCTURE WITH ADDITIONAL FEATURES TO SUPPORT THIS ANALYSIS
            each token gets an empty list to hold unit assignments
            each scope gets an empty list to hold variable unit assignments in that scope
            '''
        c = cppcheck_configuration
        for t in c.tokenlist:
            t.units = []
            t.isKnown = False
            t.is_unit_propagation_based_on_constants = False
            t.is_unit_propagation_based_on_unknown_variable = False
            t.is_unit_propagation_based_on_weak_inference = False
            t.isRoot = False
            t.hasVarOperand = False
            t.isDimensionless = False
        for s in c.scopes:
            s.var_ordered_dict = OrderedDict()
        for f in c.functions:
            f.return_units = []
            f.arg_units = []
            f.return_arg_var_nr = 0
            f.return_expr_root_token = None
            f.is_unit_propagation_based_on_constants = False
            f.is_unit_propagation_based_on_unknown_variable = False
            f.is_unit_propagation_based_on_weak_inference = False
            for arg_number in f.argument.keys():
                f.arg_units.append([])
        return c


    def init_cppcheck_config_functions(self, cppcheck_configuration):
        c = cppcheck_configuration
        for f in c.functions:
            f.maybe_generic_function = False
        return c


    def find_functions(self, a_cppcheck_configuration):
        ''' LINEAR SCAN THROUGH TOKENS TO FIND 'function' TOKENS THAT HAVE GLOBAL SCOPE.
            COLLECT AND RETURN DICT CONTAINING FUNCTION START AND END TOKEN POINTERS 
            returns: dict containing function start and end tokens
        '''
        function_dicts = {}

        # FIND FUNCTIONS IN 'SCOPES' REGION OF DUMP FILE, START AND END TOKENs
        for s in a_cppcheck_configuration.scopes:
            if s.type=='Function': 
                # SCAN ALL FUNCTIONS UNLESS LIST OF FUNCTIONS SPECIFIED
                function_dicts[s.Id] = {'name': s.className,
                                        'linern': s.classStart.linenr,
                                        'tokenStart': s.classStart, 
                                        'tokenEnd': s.classEnd, 
                                        'scopeObject':s,
                                        'symbol_table':{},
                                        'function_graph_edges':[],
                                        'function':s.function}
                # CONSTRUCT LIST OF ROOT TOKENS
                function_dicts[s.Id]['root_tokens'] = self.find_root_tokens(s.classStart, s.classEnd)
                    
        #print "Found %d functions..." % len(function_dicts)
        
        return function_dicts


    def find_root_tokens(self, tokenStart, tokenEnd):
        ''' FOR A FUNCTION DEFIND AS ALL TOKENS FROM tokenStart TO tokenEnd, FIND THE ROOTS
            input: tokenStart  a CPPCheckData Token, first token in a function
            input: tokenEnd    a CPPCheckData Token, last token in a function
            output: a list of root_tokens, in flow order
            '''
        root_tokens_set = set()
        current_token = tokenStart
        while(current_token != tokenEnd):  #todo: reverse token set exploration to top-down instead of bottom-up
            # HAS A PARENT
            if current_token.astParent: 
                a_parent = current_token.astParent
                has_parent = True
                while has_parent:
                    # HAS NO PARENT, THEREFORE IS ROOT
                    if not a_parent.astParent:     
                        root_tokens_set.add(a_parent)
                        a_parent.isRoot = True  # THIS PROPERTY IS A CUSTOM NEW PROPERTY
                        has_parent = False
                    else:
                        a_parent = a_parent.astParent 
            current_token = current_token.next

        
        root_tokens = list(root_tokens_set)
        # SORT NUMERICALLY BY LINE NUMBER
        root_tokens = sorted(root_tokens, key=lambda x : int(x.linenr))
        return root_tokens


    def collect_constraints(self, function_dict):
        tw = TreeWalker(self.type_miner, self.vnh)  
        tw.current_file = self.current_file_under_analysis
        tw.source_file_lines = self.source_file_lines
        tw.source_file = self.source_file

        # ASSUME THE TOKENS COME BACK AS A SORTED LIST
        break_point = 1000
        i=0
        found_units = False

        for root_token in function_dict['root_tokens']:
            #print root_token.str, root_token.linenr
            tw.is_unit_propagation_based_on_constants = False
            tw.is_unit_propagation_based_on_unknown_variable = False
            tw.current_child_vars = []
            tw.found_arg_var = False
            tw.found_non_arg_var = False

            # RESET THE TREE WALKER'S LINE NUMBERS
            tw.reset_min_max_line_numbers()
            # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
            tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)
            # SCAN VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.scan_variables)

            # APPLY ROS UNITS TO VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.apply_ROS_units)
            # APPLY UNITS TO KNOWN SYMBOLS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_known_symbols_units)
            # APPLY UNITS FROM KNOWN FUNCTION - getX, getY, getZ  
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_getXYZ)
            # APPLY UNITS TO CONVERSION FACTORS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_conversion_factor_units)
            # APPLY FUNCTION RETURN UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_function_return_units)
            # APPLY DIMENSIONLESS UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_dimensionless_units)

            if (tw.found_units_in_this_tree):
                found_units = True

            if (not tw.was_some_unit_changed):
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)

            
            # CONTINUE TO ATTEMPT CHANGES UNTIL CHANGES CEASE
            while tw.was_some_unit_changed:  
                if i>break_point:
                    s = "BREAKING WHILE LOOP AT %d" % break_point
                    raise ValueError(s)
                    return
                i+=1
                tw.was_some_unit_changed = False
                # LOOK FOR EARLY ABANDONMENT OF THIS AST
                if not tw.found_units_in_this_tree and self.should_abandon_early:
                    break
                ### PROPAGATE UNITS
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_dot_connectors)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_double_colon)               
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_square_brackets)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_assignment)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_abs_fabs_floor_ceil)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_min_max)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_fmod_fmodf_fmodl)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_sqrt)
                # tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_getXYZ)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_ternary)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_pow)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_inverse_trig)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_operators)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
                tw.generic_recurse_and_apply_function(root_token, tw.collect_function_param_units_and_decorate_function)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_parenthesis)
            # END -- WHILE LOOP

            #RETURN STATEMENT WITH UNITS - STORE UNITS
            if root_token.str == 'return' and root_token.units:
                if function_dict['scopeObject'].function:
                    function_return_units = function_dict['scopeObject'].function.return_units
                    for u in root_token.units:
                        if u not in function_return_units:
                            function_return_units.append(u)

            #ASSIGNMENT STATEMENT WITH UNITS - RIGHT TO LEFT PROPAGATION CONSTRAINT
            if root_token.isAssignmentOp:
                tw.collect_lhs_unit_constraint(root_token)
            
            #tw.generic_recurse_and_apply_function(root_token, tw.collect_implied_non_unit_variables)
            #SAME UNIT CONSTRAINTS
            tw.generic_recurse_and_apply_function(root_token, tw.collect_same_unit_constraints)
            tw.generic_recurse_and_apply_function(root_token, tw.collect_same_unit_constraints_II)
            tw.generic_recurse_and_apply_function(root_token, tw.collect_same_unit_constraints_III)
            #CONVERSION FACTOR CONSTRAINTS
            #tw.generic_recurse_and_apply_function(root_token, tw.collect_conversion_factor_constraints)
            tw.generic_recurse_and_apply_function(root_token, tw.collect_angle_unit_constraints)
            #KNOWN SYMBOL CONSTRAINTS
            tw.generic_recurse_and_apply_function(root_token, tw.collect_known_symbol_constraints)
            #NAMING CONSTRAINTS
            tw.generic_recurse_and_apply_function(root_token, tw.collect_naming_constraints)
        # END -- FOR LOOP 

        if (not found_units) and function_dict['scopeObject'].function \
                and ((not tw.found_var) or (tw.found_arg_or_local_var)):
                #and function_dict['scopeObject'].function.argument:
            function_dict['scopeObject'].function.maybe_generic_function = True


    def repeat_collect_constraints(self, function_dict):
        tw = TreeWalker(self.type_miner)  

        # ASSUME THE TOKENS COME BACK AS A SORTED LIST
        break_point = 1000
        i=0

        for root_token in function_dict['root_tokens']:
            #print root_token.str, root_token.linenr
            tw.is_unit_propagation_based_on_constants = False
            tw.is_unit_propagation_based_on_unknown_variable = False
            tw.current_child_vars = []
            tw.found_arg_var = False
            tw.found_non_arg_var = False

            # RESET THE TREE WALKER'S LINE NUMBERS
            tw.reset_min_max_line_numbers()
            # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
            tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)
            # SCAN VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.scan_variables)

            # APPLY ROS UNITS TO VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.apply_ROS_units)
            # APPLY UNITS TO KNOWN SYMBOLS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_known_symbols_units)
            # APPLY UNITS FROM KNOWN FUNCTION - getX, getY, getZ  
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_getXYZ)
            # APPLY UNITS TO CONVERSION FACTORS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_conversion_factor_units)
            # APPLY FUNCTION RETURN UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_function_return_units)
            # APPLY UNITS FROM PREVIOUS ROUND
            tw.generic_recurse_and_apply_function(root_token, tw.apply_previous_round_units)
            # APPLY DIMENSIONLESS UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_dimensionless_units)

            
            if (not tw.was_some_unit_changed):
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)


            # CONTINUE TO ATTEMPT CHANGES UNTIL CHANGES CEASE
            while tw.was_some_unit_changed:  
                if i>break_point:
                    s = "BREAKING WHILE LOOP AT %d" % break_point
                    raise ValueError(s)
                    return
                i+=1
                tw.was_some_unit_changed = False
                # LOOK FOR EARLY ABANDONMENT OF THIS AST
                if not tw.found_units_in_this_tree and self.should_abandon_early:
                    break
                ### PROPAGATE UNITS
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_dot_connectors)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_double_colon)                
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_square_brackets)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_assignment)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_abs_fabs_floor_ceil)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_min_max)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_fmod_fmodf_fmodl)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_sqrt)
                # tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_getXYZ)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_ternary)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_pow)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_inverse_trig)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_operators)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
                tw.generic_recurse_and_apply_function(root_token, tw.collect_function_param_units_and_decorate_function)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_parenthesis)
            # END -- WHILE LOOP

            #RETURN STATEMENT WITH UNITS - STORE UNITS
            if root_token.str == 'return' and root_token.units:
                if function_dict['scopeObject'].function:
                    function_return_units = function_dict['scopeObject'].function.return_units
                    for u in root_token.units:
                        if u not in function_return_units:
                            function_return_units.append(u)

            #ASSIGNMENT STATEMENT WITH UNITS - RIGHT TO LEFT PROPAGATION CONSTRAINT
            if root_token.isAssignmentOp:
                tw.collect_lhs_unit_constraint(root_token)
            
            #SAME UNIT CONSTRAINTS
            #tw.generic_recurse_and_apply_function(root_token, tw.collect_same_unit_constraints_III)
            #CONVERSION FACTOR CONSTRAINTS
            tw.generic_recurse_and_apply_function(root_token, tw.collect_conversion_factor_constraints)
            tw.generic_recurse_and_apply_function(root_token, tw.collect_angle_unit_constraints_II)


    def print_all_naming_constraints(self):
        for var, nm_con in con.naming_constraints.items():
            (lt, lname, units) = nm_con
            print "nm_constraint: %s %s" % (lname, units[:3])

    def print_all_computed_unit_constraints(self):
        for var, cu_con in con.computed_unit_constraints.items():
            units = []
            for (lt, lname, un, isKnown) in cu_con:
                units.append(un)
            print "cu_constraint: %s %s" % (lname, units)

        for (lt, lname, units, isKnown) in con.derived_cu_constraints:
            print "cu_constraint: %s %s" % (lname, units)

    def print_all_df_constraints(self):
        for (lt, lname, rt, rname, df_type) in con.df_constraints:
            print "df_constraint: %s %s, %s %s" % (lname, lt.units, rname, rt.units)

    def print_all_conversion_factor_constraints(self):
        for (t, name, units, cf_type) in con.conversion_factor_constraints:
            print "cf_constraint: %s %s %s" % (name, units, cf_type)

    def print_all_known_symbol_constraints(self):
        for var, ks_con in con.known_symbol_constraints.items():
            (t, name, units) = ks_con[0]
            print "ks_constraint: %s %s" % (name, units)


    def main_run_collect(self, dump_file, source_file=''): 
        ''' input: a cppcheck 'dump' file containing an Abstract Syntax Tree (AST), symbol table, and token list.
            returns: None
            side-effects: updates datbase with information about this unit analysis
        '''
        self.source_file = source_file
        self.current_file_under_analysis = dump_file
        # PARSE INPUT
        data = cppcheckdata.parsedump(dump_file)
        analysis_unit_dict = {}

        # GIVE TREE WALKER ACCESS TO SOURCE FILE FOR DEBUG PRINT
        if self.source_file and os.path.exists(self.source_file):
            with open(self.source_file) as f:
                self.source_file_lines = f.readlines()
                #print "yes"
        else:
            print "no %s %s" % (self.source_file, self.debug)

        # for c in data.configurations:  #todo: what is a data configuration?  -- Check for multiple
        for c in data.configurations[:1]:   # MODIFIED TO ONLY TEST THE FIRST CONFIGURATION
            # ADD AST DECORATION PLACEHOLDERS
            c = self.init_cppcheck_config_data_structures(c)
            c = self.init_cppcheck_config_functions(c)

            # REFRESH VARIABLES
            self.function_graph = nx.DiGraph()
            # GET DICT OF ALL GLOBALLY SCOPED FUNCTIONS
            analysis_unit_dict = self.find_functions(c)
            sorted_analysis_unit_dict = analysis_unit_dict;  # WILL BECOME AN ORDERED DICT IF self.should_sort_by_function_graph
            # FIND ORDER FOR FUNCTION GRAPH EXPLORATION ( topo sort, if possible, otherwise todo ??)
            if self.should_sort_by_function_graph:
                self.build_function_graph(analysis_unit_dict)  # WILL USE DAG SUBGRAPH
                sorted_analysis_unit_dict = self.make_sorted_analysis_unit_dict_from_function_graph(analysis_unit_dict) # RETURNS ORDERED DICT
                self.all_sorted_analysis_unit_dicts.append(sorted_analysis_unit_dict)

            # COLLECT ALL TOKEN PARSE TREES FOR EACH FUNCTION
            for function_dict in sorted_analysis_unit_dict.values():
                self.collect_constraints(function_dict)

            if self.SHOULD_PRINT_CONSTRAINTS:
                self.print_all_computed_unit_constraints()
                self.print_all_df_constraints()
                self.print_all_conversion_factor_constraints()
                self.print_all_known_symbol_constraints()
                self.print_all_naming_constraints()

            self.configurations.append(c)


    def repeat_run_collect(self, i):
        if (i > 2):
            con.is_repeat_round = True
        con.reset_constraints()

        # ASSUME ONLY ONE CONFIGURATION
        self.init_cppcheck_config_data_structures(self.configurations[0])
        sorted_analysis_unit_dict = self.all_sorted_analysis_unit_dicts[0]

        for function_dict in sorted_analysis_unit_dict.values():
            self.repeat_collect_constraints(function_dict)

        if self.SHOULD_PRINT_CONSTRAINTS:
            print "Round %d:" % i
            print "========"
            self.print_all_computed_unit_constraints()
            self.print_all_df_constraints()
            self.print_all_conversion_factor_constraints()
            self.print_all_known_symbol_constraints()
            self.print_all_naming_constraints()      


    def propagate_units(self, function_dict):
        tw = TreeWalker(self.type_miner)  

        # ASSUME THE TOKENS COME BACK AS A SORTED LIST
        break_point = 1000
        i=0

        for root_token in function_dict['root_tokens']:
            #print root_token.str, root_token.linenr
            tw.is_unit_propagation_based_on_constants = False
            tw.is_unit_propagation_based_on_unknown_variable = False
            tw.found_arg_var = False
            tw.found_non_arg_var = False

            # RESET THE TREE WALKER'S LINE NUMBERS
            tw.reset_min_max_line_numbers()
            # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
            tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)
            # SCAN VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.scan_variables)

            # APPLY ROS UNITS TO VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.apply_ROS_units)
            # APPLY UNITS TO KNOWN SYMBOLS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_known_symbols_units)
            # APPLY UNITS FROM KNOWN FUNCTION - getX, getY, getZ  
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_getXYZ)
            # APPLY UNITS TO CONVERSION FACTORS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_conversion_factor_units)
            # APPLY FUNCTION RETURN UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_function_return_units)
            # APPLY UNITS FROM PREVIOUS ROUND
            tw.generic_recurse_and_apply_function(root_token, tw.apply_previous_round_units)
            # APPLY DIMENSIONLESS UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.apply_dimensionless_units)


            if (not tw.was_some_unit_changed):
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)

            
            # CONTINUE TO ATTEMPT CHANGES UNTIL CHANGES CEASE
            while tw.was_some_unit_changed:  
                if i>break_point:
                    s = "BREAKING WHILE LOOP AT %d" % break_point
                    raise ValueError(s)
                    return
                i+=1
                tw.was_some_unit_changed = False
                # LOOK FOR EARLY ABANDONMENT OF THIS AST
                if not tw.found_units_in_this_tree and self.should_abandon_early:
                    break
                ### PROPAGATE UNITS
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_dot_connectors)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_double_colon)                
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_square_brackets)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_assignment)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_abs_fabs_floor_ceil)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_min_max)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_fmod_fmodf_fmodl)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_sqrt)
                # tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_getXYZ)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_ternary)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_pow)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_inverse_trig)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_operators)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
                tw.generic_recurse_and_apply_function(root_token, tw.collect_function_param_units_and_decorate_function)
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_parenthesis)
            # END -- WHILE LOOP

            #RETURN STATEMENT WITH UNITS - STORE UNITS
            if root_token.str == 'return' and root_token.units:
                if function_dict['scopeObject'].function:
                    function_return_units = function_dict['scopeObject'].function.return_units
                    for u in root_token.units:
                        if u not in function_return_units:
                            function_return_units.append(u)

            #ASSIGNMENT STATEMENT WITH UNITS - COLLECT MULTIPLE ASSIGNMENT INCONSISTENCY
            if root_token.isAssignmentOp:
                tw.collect_lhs_unit_constraint(root_token)


    def repeat_run_propagate(self, thresh):
        con.unit_prob_threshold = thresh
        con.reset_constraints()

        # ASSUME ONLY ONE CONFIGURATION
        self.init_cppcheck_config_data_structures(self.configurations[0])
        sorted_analysis_unit_dict = self.all_sorted_analysis_unit_dicts[0]

        for function_dict in sorted_analysis_unit_dict.values():
            self.propagate_units(function_dict)
      

    def build_function_graph(self, analysis_unit_dict):
        ''' BUILDS DIRECTED FUNCTION GRAPH 
            input:  a dictionary of functions from this dump file
            output: none.  Side effect creates a graph linked to this object
            '''
        # BUILD CALL GRAPH
        self.function_graph = nx.DiGraph()
        G = self.function_graph
        for k, function_dict in analysis_unit_dict.iteritems():
            if function_dict['function']:  # MIGHT BE NONE WHEN FUNCTION IS CLASS CONSTRUCTOR (?)
                node = function_dict['function'].Id             # Id of the Function
                #if not G.has_node(node):
                G.add_node(node) #, function_dict_key=k})  # FUNCTION CPP OBJECT IS NODE
                #else:
                all_attr = nx.get_node_attributes(G, 'function_id')
                all_attr[node] = k
                nx.set_node_attributes(G, 'function_id', all_attr)
                #function_dict['is_visited'] = False
                self.add_edges_to_function_graph(function_dict, G, node)
        self.function_graph = G


    def make_sorted_analysis_unit_dict_from_function_graph(self, analysis_unit_dict):
        ''' BUILDS A TOPO SORTED FUNCTION GRAPH.  THIS ALLOWS THE ANALYSIS TO START ON FUNCTION LEAFS, 
            SO WE CAN HOPEFULLY DISCOVER UNITS ON THE RETURN TYPE AND PROPAGE THEM UP.  THE 
            FUNCTION GRAPH MAY HAVE CYCLES (recursion, for example), THEREFORE WE REMOVE THESE EDGES FROM THE GRAPH
            AND ANALYZE THEM LAST (<-- not sure this is best)
            input:  a dictionary of functions from this dump file
            output: OrderedDict of functions
            postcondition:   returned dict must be the same length as the input dict, and contain all the same elements
            '''
        return_dict = OrderedDict()
        G = self.function_graph 
        # TRY FINDING A DAG.  IF NOT, REMOVE EDGES AND TRY AGAIN. 
        super_break = 0
        while nx.number_of_nodes(G) > 0 and super_break < 1000:
            super_break +=1 
            if not nx.is_directed_acyclic_graph(G):
                try:
                    # SEARCH FOR CYCLE AND REMOVE EDGES
                    edges = nx.find_cycle(G)
                    G.remove_edges_from(edges)
                   
                    print 'Function graph has cycle %s' % edges,
                except:
                    print 'Function graph is not a DAG and does not have a cycle!'
                    # GIVE UP AND RETURN UNSORTED 
                    return analysis_unit_dict
            else:
                # WE HAVE A DIGRAPH, CAN PROCEED ( and topo sort )
                break
        
        if nx.number_of_nodes(G) == 0:
            # RETURN UNCHANGED
            return analysis_unit_dict
        # WE HAVE A DIRECTED GRAPH WITH NODES, CAN SORT AND ADD NODES TO ORDERED LIST
        function_graph_topo_sort = nx.topological_sort(G)
        function_graph_topo_sort_reversed = function_graph_topo_sort[::-1]  # REVERSED
        
        # OPTIONAL DEBUG PRINT
        #print function_graph_topo_sort_reversed

        # CREATE RETURN DICT FROM TOPO SORT
        for node in function_graph_topo_sort_reversed:
            function_id_attr_dict = nx.get_node_attributes(G, 'function_id')
            #print function_id_attr_dict
            if node in function_id_attr_dict:  # FIRST OPTION SHOULD SHORTCUT WHEN ATTR DICT DOES NOT EXITS
                # ADD FUNCTION TO NEW DICTIONARY - THIS IS THE EXPLORE ORDER
                return_dict[function_id_attr_dict[node]] = analysis_unit_dict[function_id_attr_dict[node]]
            else:
                #print "Graph node does not have function_dict_key"
                pass
        
        # ADD ANY REMAINING FUNCTIONS NOT IN THE TOPO SORT TO THE ORDERED DICT
        for k in analysis_unit_dict.keys():
            if k not in return_dict:
                return_dict[k] = analysis_unit_dict[k]

        assert (len(return_dict) == len(analysis_unit_dict))

        return return_dict


    def debug_print_function_graph(self, analysis_unit_dict):
        if not analysis_unit_dict:
            return
        for function_dict in analysis_unit_dict.values():
            print "%s :" % function_dict['name']
            for edge in function_dict['function_graph_edges']:
                print ' --> %s' % edge.name
            

    def add_edges_to_function_graph(self, function_dict, G, current_node):
        # GET THE FIRST TOKEN AFTER THE FUNCTION DEFINITION
        current_token = function_dict['tokenStart']
        end_token = function_dict['tokenEnd']
        while current_token is not end_token: # TERMINATION GUARENTEED IF DUMP FILE IS WELL FORMED
            current_token = current_token.next  # ON FIRST LOOP, SKIPS SELF-REFERENCE 
            if current_token.function:
                if not G.has_edge(current_node, current_token.function.Id):
                    G.add_edge(current_node, current_token.function.Id)



