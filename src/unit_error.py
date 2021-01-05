from unit_error_types import UnitErrorTypes
import re


class UnitError:
    """ HELPS FIND DEFINITIONS OF SYMBOLS AND DECORATES CPPCHECK SYMBOL TABLE
    """

    def __init__(self):
        self.ERROR_TYPE = None 
        self.SEVERITY = ''
        self.var_name = ''
        self.linenr = 0
        self.was_assigned_mutiple_units = False
        # SOURCE OF UNITS
        self.are_units_from_class_definition = False
        self.are_units_from_another_variable = False
        self.are_units_from_known_function = False
        # LINE WHERE ARE UNITS ASSIGNED
        self.linenr_at_first_unit_assignment = 0
        self.linenr_of_multiple_unit_assignment = 0
        # UNITS AT FIRST ASSIGNMENT
        self.units_at_first_assignment = []
        self.source_code_at_first_assignment = '<SOURCE UNAVAILABLE>'
        self.all_units_assigned_to_var = []
        # UNITS WHEN MULTIPLE ASSIGNMENT HAPPENED
        self.units_when_multiple_happened = []
        self.source_code_when_multiple_units_happened = '<SOURCE UNAVAILABLE>'
        # UNITS WHEN COMPARISON ON INCOMPATIBLE UNITS
        self.token_left = None
        self.token_right = None
        self.token = None
        # UNIT INFERENCE STRENGTH
        self.is_warning = False
        self.is_unit_propagation_based_on_unknown_variable = False
        self.is_unit_propagation_based_on_constants = False
        self.dont_check_for_warning = False


    def set_primary_line_number(self, linenr):
        ''' SETS LINE NUMBER FOR PRIMARY LOCATION OF ERROR
            input: line number (assume str, will also work for int)
            returns: None
            side_effect(s):  sets object properties
            '''
        my_linenr = int(linenr)
        self.linenr = my_linenr
        if self.ERROR_TYPE in \
                [UnitErrorTypes.VARIABLE_MULTIPLE_UNITS,
                 UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS]:
            self.linenr_at_first_unit_assignment = my_linenr


    def get_error_desc(self):
        return UnitErrorTypes().get_err_short_discription(self.ERROR_TYPE)


