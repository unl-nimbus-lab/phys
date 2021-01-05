import cps_constraints as con
import copy


class SymbolHelper:
    ''' HELPS FIND DEFINITIONS OF SYMBOLS AND DECORATES CPPCHECK SYMBOL TABLE
    '''

    def __init__(self):
        self.ros_unit_dictionary = {}
        self.should_ignore_time_and_math = False
        self.should_use_dt_heuristic = True
        self.initialize_ros_unit_dictionary()      
        self.debug_missed_class_names_output_file = 'all_missed_class_name_lookups.txt'
        self.debug_log_missed_class_names = False
        self.is_weak_inference = False
        self.weak_inference_classes = ['sensor_msgs::JointState', 'trajectory_msgs::JointTrajectory', 'trajectory_msgs::JointTrajectoryPoint']
        # THESE DIMENSIONLESS UNITS ACT AS A '1' DURTION DIMENSION OPERATIONS, EXEPCT FOR ADDITION.  
        # EXAMPLES    radians + meters NOT OK.  radians * meters = meters.  quaternion * quaternion = quaternion
        self.dimensionless_units = [{'radian': 1.0}, {'quaternion': 1.0}, {'nounit': 0.0}, {'degree_360': 1.0}]
        self.dimensionless_units_as_lists = [[{'radian': 1.0}], [{'quaternion': 1.0}], [{'nounit': 0.0}], [{'degree_360': 1.0}]]


    def should_have_unit(self, token, name):
        if token.variable:
            if con.is_int_unit_variable(token, name):
                return True

            if con.is_non_unit_variable(token, name):
                return False

            if name in ['argc', 'argv']:
                return False

            # ROS common messages format members
            if any(substr in name for substr in ['header.frame_id', 'header.seq', '.name', '.child_frame_id']):
                return False

            #if token.variable.isClass and name == token.str:
            #    return False
           
            var_type = self.find_variable_type(token.variable)
            var_type = self.sanitize_class_name(var_type)
            var_type = var_type.lower()
            if name == token.str:
                if not(var_type in ['ros::time', 'ros::duration', 'std::vector<ros::duration>', 'ros::rate']) and \
                        (not(any(substr in var_type for substr in ['float', 'double']))):
                        # any(substr in var_type for substr in ['point', 'joint'])):
                        # JPO:  might be too strong to exclude variables like 'distance_to_point' or 'joint_torque_limit'
                    return False    

            if var_type in ["bool", "std::string", "string"]:
                return False

            if ('bool' in var_type or
                    'Bool' in var_type or
                    'Byte' in var_type or
                    'int' == var_type or
                    'int32' in var_type or
                    'int16' in var_type or
                    'uint32_t' == var_type or
                    'size_t' in var_type or
                    'uint' in var_type[:4] or
                    'char' == var_type or
                    'ros::NodeHandle' == var_type or
                    'ROSAgent' == var_type or
                    'ros::ServiceClient' == var_type or
                    'OrientToBaseResult' == var_type or
                    'OrientToBaseGoal' == var_type or
                    #'ros::Rate' == var_type or
                    'OrientToLaserReadingAction' in var_type or
                    'ServiceServer' in var_type or
                    'Subscriber' in var_type or
                    'Publisher' in var_type or
                    'string' in var_type or
                    'actionlib' in var_type or
                    'IStream' in var_type or
                    'rosserial_msgs' in var_type or
                    'TransformException' in var_type or
                    'TransformBroadcaster' in var_type or
                    'TransformListener' in var_type or
                    'tf::Vector3' in var_type or
                    'OrientToBaseAction' in var_type):
                    # or
                    # '::' in class_name):
                return False


        elif token.isString:
            return False

        elif name.lower() in ['true', 'false']:
            return False

        return True


    def find_compound_variable_and_name_for_variable_token(self, token):
        if not token.variable:
            raise ValueError('received a non variable token for tokenid:%s str:%s' % (token.Id, token.str))
        if not token.astParent:
            return (token, token.str)
        if token.astParent.str != '.' and token.astParent.str != '[':
            return (token, token.str)
        compound_variable_token = token
        compound_variable_root_token = token
        while compound_variable_root_token.astParent and \
                (compound_variable_root_token.astParent.str == '.' or compound_variable_root_token.astParent.str == '['):
            if compound_variable_root_token.astParent.astOperand2:
                if compound_variable_root_token.astParent.astOperand2.variable and \
                        compound_variable_root_token.astParent.str != '[':
                    compound_variable_token = compound_variable_root_token.astParent.astOperand2
                #if compound_variable_root_token.astParent.astOperand2.varId:
                #    compound_variable_root_token = compound_variable_root_token.astParent
                #else:
                #    break
                compound_variable_root_token = compound_variable_root_token.astParent
            else:
                break

        if compound_variable_root_token.astParent and compound_variable_root_token.astParent.str == '(' and \
                compound_variable_root_token.astParent.astOperand1 and \
                        compound_variable_root_token.astParent.astOperand1 == compound_variable_root_token:
            var_type = self.find_variable_type(compound_variable_token.variable)
            var_type = var_type.lower()
            if 'vector' in var_type:
                if compound_variable_root_token.astOperand2 and compound_variable_root_token.astOperand2.str == 'at':
                    name = self.recursively_visit(compound_variable_root_token.astOperand1)
                    return (compound_variable_token, name)
                
            # FUNCTION CALL
            return (None, '')

        name = self.recursively_visit(compound_variable_root_token)
        return (compound_variable_token, name)
        

    #TODO check which variable token to be returned
    def find_compound_variable_and_name_for_dot_operand(self, token):
        if token.str != '.' and token.str != '[' and token.str != '(':
            print 'received a non dot token for tokenid:%s str:%s' % (token.Id, token.str)
            return (token, token.str)

        compound_variable_token = token
        if token.str == '(' and token.astOperand1:
            compound_variable_token = token.astOperand1

        while compound_variable_token.astOperand1 and (not compound_variable_token.variable):
            if compound_variable_token.astOperand2 and compound_variable_token.astOperand2.variable and \
                    compound_variable_token.str != '[':
                compound_variable_token = compound_variable_token.astOperand2
            else:
                compound_variable_token = compound_variable_token.astOperand1

        if token.str == '(':
            if compound_variable_token.variable:
                var_type = self.find_variable_type(compound_variable_token.variable)
                var_type = var_type.lower()
                if 'vector' in var_type:
                    if token.astOperand1 and token.astOperand1.str == '.' and \
                            token.astOperand1.astOperand2 and token.astOperand1.astOperand2.str == 'at':
                        name = self.recursively_visit(token.astOperand1.astOperand1)
                        return (compound_variable_token, name)

            # FUNCTION CALL
            return (token, token.str)

        name = self.recursively_visit(token)
        return (compound_variable_token, name)


    def find_compound_variable_name_for_variable_token(self, token):
        ''' input: a variable token from cppcheckdata
            returns: string containing the compound variable name.  ex:  'my_var_.linear.x'
            '''
        if not token.variable:
            raise ValueError('received a non variable token for tokenid:%s str:%s' % (token.Id, token.str))
        if not token.astParent:
            return token.str
        if token.astParent.str != '.':
            return token.str
        compound_variable_root_token = token
        while compound_variable_root_token.astParent and compound_variable_root_token.astParent.str == '.':
            if compound_variable_root_token.astParent.astOperand2 and compound_variable_root_token.astParent.astOperand2.varId:
                compound_variable_root_token = compound_variable_root_token.astParent
            else:
                break
        return self.recursively_visit(compound_variable_root_token)


    def find_compound_variable_name_for_ros_variable_token(self, token):
        ''' input: a variable token from cppcheckdata
            returns: string containing the compound variable name.  ex:  'my_var_.linear.x'
            '''
        if not token.variable:
            raise ValueError('received a non variable token for tokenid:%s str:%s' % (token.Id, token.str))
        if token.astParent.str != '.':
            return token.str
        compound_variable_root_token = token.astParent
        while compound_variable_root_token.astParent and compound_variable_root_token.astParent.str == '.':
            compound_variable_root_token = compound_variable_root_token.astParent
        return self.recursively_visit(compound_variable_root_token)

            
    def recursively_visit(self, token):
        ''' input: a cppcheckdata token
            returns: string aggregation of tokens under the root
            '''
        my_return_string = ''

        if token.astOperand1: 
            my_return_string += self.recursively_visit(token.astOperand1)
        my_return_string += token.str 
        if token.astOperand2:
            my_return_string += self.recursively_visit(token.astOperand2)
        if token.str == '[':
            my_return_string += ']'
        
        return my_return_string


    def find_variable_type(self, variable):
        ''' input: cppcheckdata variable object
            output: string of variable type  (ex:  'int'  or 'std::vector')
            '''
        my_return_string = variable.typeStartToken.str

        if variable.typeStartToken != variable.typeEndToken:
            nextToken = variable.typeStartToken.next
            hasNext = True
            while(hasNext and nextToken):  # BREAK at end of type region
                my_return_string += nextToken.str
                if nextToken.Id == variable.typeEndTokenId:
                    hasNext = False;
                nextToken = nextToken.next

        return my_return_string


    def rreplace(self, s, old, new, occurance):
        li = s.rsplit(old, occurance)
        return new.join(li)


    def sanitize_class_name(self, class_name):
        ''' TAKES A VARIABLE NAME AND STRIPTS EXTRA CLASS INFORMATION FROM FRONT AND BACK
            input: class_name   'constPtr<tf2::Transform::iterator>'  <-- LOL
            output: str         'trf2::Transform'
            '''
        is_some_change = True
        while (is_some_change):# SOME OF THESE CAN BE CASCADED IN DIFFERENT ORDERS
            is_some_change = False

            # SIMPLIFY ITERATORS
            if str(class_name).endswith('::iterator'):
                class_name = self.rreplace(class_name, '::iterator', '', 1)
                is_some_change = True
            if str(class_name).endswith('::const_iterator'):
                class_name = self.rreplace(class_name, '::const_iterator', '', 1)
                is_some_change = True
            # SIMPLIFY REFERNCES
            if str(class_name).endswith('&'):
                class_name = class_name.replace('&', '')
                is_some_change = True
            # SIMPLIFY CONST
            if str(class_name).endswith('const'):
                class_name = self.rreplace(class_name, 'const', '', 1)
                is_some_change = True
            # SIMPLIFY CONSTANT POINTERS (REFRENCES)
            if str(class_name).endswith('ConstPtr'):
                class_name = class_name.replace('ConstPtr', '')
                is_some_change = True
            # SIMPLIFY POINTERS
            if str(class_name).endswith('::Ptr'):
                class_name = self.rreplace(class_name, '::Ptr', '', 1)
                is_some_change = True
            if str(class_name).endswith('Ptr'):
                class_name = self.rreplace(class_name, 'Ptr', '', 1)
                is_some_change = True
            # SIMPLIFY BOOST POINTERS
            if str(class_name).startswith('boost::shared_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('boost::shared_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('shared_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('shared_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('boost::scoped_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('boost::scoped_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('scoped_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('scoped_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True

            if str(class_name).startswith('tf::MessageFilter<') and str(class_name).endswith('>'):
                class_name = class_name.replace('tf::MessageFilter<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('tf2_ros::MessageFilter<') and str(class_name).endswith('>'):
                class_name = class_name.replace('tf_ros::MessageFilter<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('MessageFilter<') and str(class_name).endswith('>'):
                class_name = class_name.replace('MessageFilter<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True

            # SIMPLIFY VECTORS
            if str(class_name).startswith('std::vector<') and str(class_name).endswith('>'):
                class_name = class_name.replace('std::vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('conststd::vector<') and str(class_name).endswith('>'):
                class_name = class_name.replace('std::vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('vector<') and str(class_name).endswith('>'):
                class_name = class_name.replace('vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('std::deque<') and str(class_name).endswith('>'):
                class_name = class_name.replace('std::vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('deque<') and str(class_name).endswith('>'):
                class_name = class_name.replace('vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            # SIMPLIFY POINTERS
            if str(class_name).endswith('*'):
                class_name = self.rreplace(class_name, '*', '', 1)
                is_some_change = True
            # SIMPLIFY DANGLING ::
            if str(class_name).endswith('::'):
                class_name = self.rreplace(class_name, '::', '', 1)
                is_some_change = True
        # SANITIZED VERSION
        return class_name


    def find_units_for_variable(self, token):
        ''' input:  token       a cppcheckdata token that contains a variable 
            returns: a dictionary of unit types
            '''
        # ASSUME FALSE
        self.is_weak_inference = False

        raw_class_name = self.find_variable_type(token.variable)

        # REMOVE EXTRA, UNNECESSARY INFORMATION FROM FRONT AND BACK
        class_name = self.sanitize_class_name(raw_class_name)

        # IF CLASS IS A ROS MESSAGE
        if class_name in self.ros_unit_dictionary:
            # FIND POSSIBLE ATTRIBUTES
            possible_attributes_as_list = self.find_compound_variable_name_for_ros_variable_token(token).split(".")
            my_return_units = None
            if possible_attributes_as_list:
                # STARTED AS SPECIAL CASE FOR LaserScan
                # CORNER CASES WHEN WE DONT' ASSIGN UNITS
                # if (class_name == 'sensor_msgs::LaserScan') and 'size' in possible_attributes_as_list:
                if 'size' == possible_attributes_as_list[-1]:
                    return {}
                if 'empty' == possible_attributes_as_list[-1]:
                    return {}
                if 'isZero' ==  possible_attributes_as_list[-1]:
                    return {}
                # THE USUAL EXPECTED CASE 
                for p in possible_attributes_as_list:
                    if p in self.ros_unit_dictionary[class_name]:
                        my_return_units = self.ros_unit_dictionary[class_name][p]
            if not self.should_ignore_time_and_math:
                if class_name in ['ros::Time', 'ros::Duration', 'std::vector<ros::Duration>']:
                    my_return_units = self.ros_unit_dictionary[class_name]['sec']
                elif class_name in ['ros::Rate']:
                    my_return_units = self.ros_unit_dictionary[class_name]['rate']
            if my_return_units:
                # CHECK FOR WEAK INFERENCE CLASSES, SUCH AT JOINT_STATE BECAUSE THAT COULD BE r/s or m/s
                if class_name in self.weak_inference_classes:
                    self.is_weak_inference = True
                return my_return_units
        else:
            if self.should_use_dt_heuristic and not self.should_ignore_time_and_math:
                if token.str.lower() == 'dt':
                    #self.is_weak_inference = True
                    return self.ros_unit_dictionary['dt']['dt']
            if self.debug_log_missed_class_names:
                with open(self.debug_missed_class_names_output_file, 'a') as f:
                    f.write(class_name + '\n')
        # REACHNG HERE MEANS WE FOUND NOTHING
        return {}


    def initialize_ros_unit_dictionary(self):


        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  WRENCH
        twist_dict                                      = {'linear': {'meter': 1.0, 'second': -1.0}, 
                                                           'angular': {'second': -1.0}}
        accel_dict                                      = {'linear': {'meter': 1.0, 'second': -2.0}, 
                                                           'angular': {'second': -2.0}}
                                                          #'covariance'} todo
        wrench_dict                                     = {'force': {'kilogram': 1.0, 'meter': 1.0, 'second': -2.0}, 
                                                           'torque': {'kilogram': 1.0, 'meter': 2.0, 'second': -2.0}}
        inertia_dict                                    = {'m': {'kilogram': 1.0},
                                                           'com': {'meter': 1.0},
                                                           'ixx': {'kilogram': 1.0, 'meter': -2.0},
                                                           'ixy': {'kilogram': 1.0, 'meter': -2.0},
                                                           'ixz': {'kilogram': 1.0, 'meter': -2.0},
                                                           'iyy': {'kilogram': 1.0, 'meter': -2.0},
                                                           'iyz': {'kilogram': 1.0, 'meter': -2.0},
                                                           'izz': {'kilogram': 1.0, 'meter': -2.0}}
        transform_dict                                  = {'translation': {'meter': 1.0},
                                                           'rotation': {'quaternion': 1.0}}


    # GEOMETRY MSGS   http://docs.ros.org/api/geometry_msgs/html/index-msg.html
        
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  ACCEL
        self.ros_unit_dictionary['geometry_msgs::Accel'] = accel_dict
        self.ros_unit_dictionary['geometry_msgs::AccelStamped'] = copy.deepcopy(accel_dict)
        self.ros_unit_dictionary['geometry_msgs::AccelStamped']['stamp'] = {'second': 1.0}
        self.ros_unit_dictionary['geometry_msgs::AccelWithCovariance'] = accel_dict
        self.ros_unit_dictionary['geometry_msgs::AccelWithCovarianceStamped'] = copy.deepcopy(accel_dict)
        self.ros_unit_dictionary['geometry_msgs::AccelWithCovarianceStamped']['stamp'] = {'second': 1.0}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  INERTIA
        self.ros_unit_dictionary['geometry_msgs::Inertia'] = inertia_dict
        self.ros_unit_dictionary['geometry_msgs::InertiaStamped'] = copy.deepcopy(inertia_dict)
        self.ros_unit_dictionary['geometry_msgs::InertiaStamped']['stamp'] = {'second': 1.0}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT
        self.ros_unit_dictionary['geometry_msgs::Point'] = {'x': {'meter': 1.0},
                                                            'y': {'meter': 1.0},
                                                            'z': {'meter': 1.0}}
        self.ros_unit_dictionary['std::vector<geometry_msgs::Point>'] = {'x': {'meter': 1.0},
                                                                         'y': {'meter': 1.0},
                                                                         'z': {'meter': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT
        self.ros_unit_dictionary['geometry_msgs::Point32'] = {'x': {'meter': 1.0},
                                                              'y': {'meter': 1.0},
                                                              'z': {'meter': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT
        self.ros_unit_dictionary['geometry_msgs::PointStamped'] = {'x': {'meter': 1.0},
                                                                   'y': {'meter': 1.0},
                                                                   'z': {'meter': 1.0},
                                                                   'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['geometry_msgs::PointStampedPtr'] = {'x': {'meter': 1.0},
                                                                      'y': {'meter': 1.0},
                                                                      'z': {'meter': 1.0},
                                                                      'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['MessageFilter<geometry_msgs::PointStamped>'] = {'x': {'meter': 1.0},
                                                                                  'y': {'meter': 1.0},
                                                                                  'z': {'meter': 1.0},
                                                                                  'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['std::vector<geometry_msgs::PointStamped>'] = {'x': {'meter': 1.0},
                                                                                'y': {'meter': 1.0},
                                                                                'z': {'meter': 1.0},
                                                                                'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POLYGON
        self.ros_unit_dictionary['geometry_msgs::Polygon'] = {'points': {'meter': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POLYGONSTAMPED
        self.ros_unit_dictionary['geometry_msgs::PolygonStamped'] = {'points': {'meter': 1.0},
                                                                     'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSE
        self.ros_unit_dictionary['geometry_msgs::Pose'] = {'position': {'meter': 1.0},
                                                           'orientation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['std::Vector<geometry_msgs::Pose>'] = {'position': {'meter': 1.0},
                                                                        'orientation': {'quaternion': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSE
        self.ros_unit_dictionary['geometry_msgs::Pose2D'] = {'x': {'meter': 1.0},
                                                             'y': {'meter': 1.0},
                                                             'theta': {'radian': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSESTAMPED
        self.ros_unit_dictionary['geometry_msgs::PoseStamped'] = {'position': {'meter': 1.0},
                                                                  'orientation': {'quaternion': 1.0},
                                                                  'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['std::Vector<geometry_msgs::PoseStamped>'] = {'position': {'meter': 1.0},
                                                                               'orientation': {'quaternion': 1.0},
                                                                               'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSEWITHCOVARIANCE
        self.ros_unit_dictionary['geometry_msgs::PoseWithCovariance'] = {'position': {'meter': 1.0},
                                                                         'orientation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['geometry_msgs::PoseWithCovariance::_covariance_type'] = {'position': {'meter': 1.0},
                                                                                           'orientation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['geometry_msgs::PoseWithCovarianceStamped'] = {'position': {'meter': 1.0},
                                                                                'orientation': {'quaternion': 1.0},
                                                                                'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSE ARRAY  todo
        self.ros_unit_dictionary['geometry_msgs::PoseArray'] = {'position': {'meter': 1.0},
                                                                'orientation': {'quaternion': 1.0},
                                                                'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  Quaternion
        self.ros_unit_dictionary['geometry_msgs::Quaternion'] = {'x': {'quaternion': 1.0},
                                                                 'y': {'quaternion': 1.0},
                                                                 'z': {'quaternion': 1.0},
                                                                 'w': {'quaternion': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  QUATERNION STAMPED
        self.ros_unit_dictionary['geometry_msgs::QuaternionStamped'] = {'x': {'quaternion': 1.0},
                                                                        'y': {'quaternion': 1.0},
                                                                        'z': {'quaternion': 1.0},
                                                                        'w': {'quaternion': 1.0},
                                                                        'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TRANSFORM
        self.ros_unit_dictionary['geometry_msgs::Transform'] = transform_dict
        self.ros_unit_dictionary['geometry_msgs::TransformStamped'] = copy.deepcopy(transform_dict)
        self.ros_unit_dictionary['geometry_msgs::TransformStamped']['stamp'] = {'second': 1.0}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TWIST
        self.ros_unit_dictionary['geometry_msgs::Twist'] = twist_dict
        self.ros_unit_dictionary['geometry_msgs::TwistStamped'] = copy.deepcopy(twist_dict)
        self.ros_unit_dictionary['geometry_msgs::TwistStamped']['stamp'] = {'second': 1.0}
        self.ros_unit_dictionary['geometry_msgs::TwistWithCovariance'] = copy.deepcopy(twist_dict)
        self.ros_unit_dictionary['geometry_msgs::TwistWithCovarianceStamped'] = copy.deepcopy(twist_dict)
        self.ros_unit_dictionary['geometry_msgs::TwistWithCovarianceStamped']['stamp'] = {'second': 1.0}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  WRENCH
        self.ros_unit_dictionary['geometry_msgs::Wrench'] = wrench_dict
        self.ros_unit_dictionary['geometry_msgs::WrenchStamped'] = copy.deepcopy(wrench_dict)
        self.ros_unit_dictionary['geometry_msgs::WrenchStamped']['stamp'] = {'second': 1.0}


    # NAVIGATION MSGS   http://docs.ros.org/api/nav_msgs/html/index-msg.html

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  GRID CELLS
        self.ros_unit_dictionary['nav_msgs::GridCells'] = {'cell_width': {'meter': 1.0}, 
                                                           'cell_height': {'meter': 1.0},
                                                           'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MAP META DATA
        self.ros_unit_dictionary['nav_msgs::MapMetaData'] = {'map_load_time': {'second': 1.0},
                                                             'resolution': {'meter': 1.0},
                                                             'x': {'meter': 1.0},
                                                             'y': {'meter': 1.0},
                                                             'z': {'radian': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  OCCUPANCY GRID
        self.ros_unit_dictionary['nav_msgs::OccupancyGrid'] = {'map_load_time': {'second': 1.0},
                                                               'resolution': {'meter': 1.0},
                                                               'x': {'meter': 1.0},
                                                               'y': {'meter': 1.0},
                                                               'z': {'radian': 1.0},
                                                               'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  ODOMETRY
        self.ros_unit_dictionary['nav_msgs::Odometry'] = {'position': {'meter': 1.0},
                                                          'orientation': {'quaternion': 1.0},
                                                          'stamp': {'second': 1.0},
                                                          'linear': {'meter': 1.0, 'second': -1.0},
                                                          'angular': {'second': -1.0}}

        self.ros_unit_dictionary['nav_msgs::Path'] = {'position': {'meter': 1.0},
                                                      'orientation': {'quaternion': 1.0},
                                                      'stamp': {'second': 1.0}}


    # NAV 2D MSGS
        self.ros_unit_dictionary['nav_2d_msgs::Twist2D'] = {'x': {'meter': 1.0, 'second': -1.0}, 
                                                            'y': {'meter': 1.0, 'second': -1.0}, 
                                                            'theta': {'radian': 1.0}}
        self.ros_unit_dictionary['nav_2d_msgs::Twist2D32'] = {'x': {'meter': 1.0, 'second': -1.0}, 
                                                              'y': {'meter': 1.0, 'second': -1.0}, 
                                                              'theta': {'radian': 1.0}}
        self.ros_unit_dictionary['nav_2d_msgs::Twist2D32Stamped'] = {'x': {'meter': 1.0, 'second': -1.0}, 
                                                                     'y': {'meter': 1.0, 'second': -1.0}, 
                                                                     'theta': {'radian': 1.0}}


    # SENSOR MSGS   http://docs.ros.org/api/sensor_msgs/html/index-msg.html

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  BATTERY   todo: not implemented in Indigo
        self.ros_unit_dictionary['sensor_msgs::BatteryState'] = {'voltage': {'kilogram': 1.0, 'meter': 2.0, 'second': -3.0, 'amp': -1.0},
                                                                 'cell_voltage': {'kilogram': 1.0, 'meter': 2.0, 'second': -3.0, 'amp': -1.0},
                                                                 'current': {'amp': 1.0},
                                                                 'charge': {'amp': 1.0, 'second': 1.0},
                                                                 'capacity': {'amp': 1.0, 'second': 1.0},
                                                                 'design_capacity': {'amp': 1.0, 'second': 1.0},
                                                                 'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  FLUID PRESSURE 
        self.ros_unit_dictionary['sensor_msgs::FluidPressure'] = {'fluid_pressure': {'kilogram': 1.0, 'meter': -1.0, 'second': -2.0},
                                                                  'variance': {'kilogram': 2.0, 'meter': -2.0, 'second': -4.0},
                                                                  'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  ILLUMINANCE
        self.ros_unit_dictionary['sensor_msgs::Illuminance'] = {'illuminance': {'candela': 1.0, 'meter': -2.0},
                                                                'variance': {'candela': 2.0, 'meter': -4.0},
                                                                'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  IMU
        self.ros_unit_dictionary['sensor_msgs::Imu'] = {'angular_velocity': {'second': -1.0},
                                                        'angular_velocity_covariance': {'second': -2.0},
                                                        'linear_acceleration': {'meter': 1.0, 'second': -2.0},
                                                        'linear_acceleration_covariance': {'meter': 2.0, 'second': -4.0},
                                                        'orientation': {'quaternion': 1.0},
                                                        'orientation_covariance': {'quaternion': 2.0},
                                                        'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  JOINT STATE  #todo: Improve on revolute assumption
        self.ros_unit_dictionary['sensor_msgs::JointState'] = {'position': {'radian': 1.0},
                                                               'velocity': {'second': -1.0},
                                                               'effort': {'kilogram': 1.0, 'meter': 2.0, 'second': -2.0},
                                                               'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  LASER ECHO
        self.ros_unit_dictionary['sensor_msgs::LaserEcho'] = {'echoes': {'meter': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  LASERSCAN
        self.ros_unit_dictionary['sensor_msgs::LaserScan'] = {'time_increment': {'second': 1.0},
                                                              'scan_time': {'second': 1.0},
                                                              'range_min': {'meter': 1.0},
                                                              'range_max': {'meter': 1.0},
                                                              'ranges': {'meter': 1.0},
                                                              'angle_min': {'radian': 1.0},
                                                              'angle_max': {'radian': 1.0},
                                                              'angle_increment': {'radian': 1.0},
                                                              'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MAGNETIC FIELD
        self.ros_unit_dictionary['sensor_msgs::MagneticField'] = {'magnetic_field': {'kilogram': 1.0, 'second': -2.0, 'amp': -1.0},
                                                                  'magnetic_field_covariance': {'kilogram': 2.0, 'second': -4.0, 'amp': -2.0},
                                                                  'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MULTI DOF JOINT STATE
        self.ros_unit_dictionary['sensor_msgs::MultiDOFJointState'] = {'twist': twist_dict,
                                                                       'wrench': wrench_dict,
                                                                       'transforms': transform_dict,
                                                                       'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MULTI ECHO LASERSCAN
        self.ros_unit_dictionary['sensor_msgs::MultiEchoLaserScan'] = {'time_increment': {'second': 1.0},
                                                                       'scan_time': {'second': 1.0},
                                                                       'range_min': {'meter': 1.0},
                                                                       'range_max': {'meter': 1.0},
                                                                       'ranges': {'meter': 1.0},
                                                                       'angle_min': {'radian': 1.0},
                                                                       'angle_max': {'radian': 1.0},
                                                                       'angle_increment': {'radian': 1.0},
                                                                       'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  NAV SAT FIX
        self.ros_unit_dictionary['sensor_msgs::NavSatFix'] = {'latitude': {'degree_360': 1.0},
                                                              'longitude': {'degree_360': 1.0},
                                                              'altitude': {'meter': 1.0},
                                                              'position_covariance': {'meter': 2.0},
                                                              'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT CLOUD
        self.ros_unit_dictionary['sensor_msgs::PointCloud'] = {'points': {'meter': 1.0},
                                                               'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['sensor_msgs::PointCloud2'] = {'points': {'meter': 1.0},
                                                                'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['sensor_msgs::PointCloud2Iterator<float>'] = {'points': {'meter': 1.0},
                                                                               'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  RANGE
        self.ros_unit_dictionary['sensor_msgs::Range'] = {'min_range': {'meter': 1.0},
                                                          'max_range': {'meter': 1.0},
                                                          'range': {'meter': 1.0},
                                                          'stamp': {'second': 1.0},
                                                          'field_of_view': {'radian': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TEMPERATURE
        self.ros_unit_dictionary['sensor_msgs::Temperature'] = {'temperature': {'degree_celsius': 1.0},
                                                                'variance': {'degree_celsius': 2.0},
                                                                'stamp': {'second': 1.0}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TIME REFERENCE
        self.ros_unit_dictionary['sensor_msgs::TimeReference'] = {'time_ref': {'second': 1.0},
                                                                  'stamp': {'second': 1.0}}


    # SHAPE MSGS   http://docs.ros.org/api/shape_msgs/html/index-msg.html

        self.ros_unit_dictionary['shape_msgs::Mesh'] = {'vertices': {'meter': 1.0}}

        self.ros_unit_dictionary['shape_msgs::SolidPrimitive'] = {'dimensions': {'meter': 1.0}}


    # STEREO MSGS   http://docs.ros.org/api/stereo_msgs/html/index-msg.html

        self.ros_unit_dictionary['stereo_msgs::DisparityImage'] = {'T': {'meter': 1.0}, 
                                                                   #'min_disparity': {'meter': 1.0},
                                                                   #'delta_d': {'meter': 1.0},
                                                                   #'max_disparity': {'meter': 1.0},
                                                                   'stamp': {'second': 1.0}}


    # TRAJECTORY MSGS   http://wiki.ros.org/trajectory_msgs
        # REVOLUTE ASSUMPTION LIKE JOINTSTATE
        self.ros_unit_dictionary['trajectory_msgs::JointTrajectory'] = {'positions': {'radian': 1.0}, 
                                                                        'velocities': {'second': -1.0},
                                                                        'accelerations': {'second': -2.0},
                                                                        'effort': {'kilogram': 1.0, 'meter': 2.0, 'second': -2.0},
                                                                        'time_from_start': {'second': 1.0},
                                                                        'stamp': {'second': 1.0}}

        self.ros_unit_dictionary['trajectory_msgs::JointTrajectoryPoint'] = {'positions': {'radian': 1.0}, 
                                                                        'velocities': {'second': -1.0},
                                                                        'accelerations': {'second': -2.0},
                                                                        'effort': {'kilogram': 1.0, 'meter': 2.0, 'second': -2.0},
                                                                        'time_from_start': {'second': 1.0}}
        

    # TRANSFORM MSGS

        self.ros_unit_dictionary['StampedTransform'] = {'getOrigin': {'meter': 1.0}, 
                                                        'getRotation': {'quaternion': 1.0},
                                                        'stamp_': {'second': 1.0}}

        self.ros_unit_dictionary['tf::Quaternion'] = {'getX': {'quaternion': 1.0}, 
                                                      'getY': {'quaternion': 1.0},
                                                      'getZ': {'quaternion': 1.0},
                                                      'getW': {'quaternion': 1.0}}
        self.ros_unit_dictionary['tf::Stamped<tf::Quaternion>'] = {'getX': {'quaternion': 1.0}, 
                                                                   'getY': {'quaternion': 1.0},
                                                                   'getZ': {'quaternion': 1.0},
                                                                   'getW': {'quaternion': 1.0},
                                                                   'stamp_': {'second': 1.0}}

        #self.ros_unit_dictionary['tf::Vector3'] = {'getX': {'meter': 1.0}, 
        #                                           'getY': {'meter': 1.0},
        #                                           'getZ': {'meter': 1.0},
        #                                           'x': {'meter': 1.0},
        #                                           'y': {'meter': 1.0},
        #                                           'z': {'meter': 1.0}}
        self.ros_unit_dictionary['tf::Stamped<tf::Vector3>'] = {#'getX': {'meter': 1.0}, 
                                                                #'getY': {'meter': 1.0},
                                                                #'getZ': {'meter': 1.0},
							        #'x': {'meter': 1.0},
							        #'y': {'meter': 1.0},
							        #'z': {'meter': 1.0},
                                                                'stamp_': {'second': 1.0}}

        self.ros_unit_dictionary['tf::Pose'] = {'getOrigin': {'meter': 1.0}, 
                                                'getRotation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['tf::Stamped<tf::Pose>'] = {'getOrigin': {'meter': 1.0}, 
                                                             'getRotation': {'quaternion': 1.0},
                                                             'stamp_': {'second': 1.0}}

        self.ros_unit_dictionary['tf::Transform'] = {'getOrigin': {'meter': 1.0}, 
                                                     'getRotation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['tf::StampedTransform'] = {'getOrigin': {'meter': 1.0}, 
                                                            'getRotation': {'quaternion': 1.0},
                                                            'stamp_': {'second': 1.0}}
        self.ros_unit_dictionary['tf2::Stamped<tf::Transform>'] = {'getOrigin': {'meter': 1.0}, 
                                                                   'getRotation': {'quaternion': 1.0},
                                                                   'stamp_': {'second': 1.0}}

        self.ros_unit_dictionary['tf2::Quaternion'] = {'getX': {'quaternion': 1.0}, 
                                                       'getY': {'quaternion': 1.0},
                                                       'getZ': {'quaternion': 1.0},
                                                       'getW': {'quaternion': 1.0}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Quaternion>'] = {'getX': {'quaternion': 1.0}, 
                                                                     'getY': {'quaternion': 1.0},
                                                                     'getZ': {'quaternion': 1.0},
                                                                     'getW': {'quaternion': 1.0},
                                                                     'stamp_': {'second': 1.0}}

        #self.ros_unit_dictionary['tf2::Vector3'] = {'getX': {'meter': 1.0}, 
        #                                            'getY': {'meter': 1.0},
        #                                            'getZ': {'meter': 1.0}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Vector3>'] = {#'getX': {'meter': 1.0}, 
                                                                  #'getY': {'meter': 1.0},
                                                                  #'getZ': {'meter': 1.0},
                                                                  'stamp_': {'second': 1.0}}

        self.ros_unit_dictionary['tf2::Pose'] = {'getOrigin': {'meter': 1.0}, 
                                                 'getRotation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Pose>'] = {'getOrigin': {'meter': 1.0}, 
                                                               'getRotation': {'quaternion': 1.0},
                                                               'stamp_': {'second': 1.0}}

        self.ros_unit_dictionary['tf2::Transform'] = {'getOrigin': {'meter': 1.0}, 
                                                      'getRotation': {'quaternion': 1.0}}
        self.ros_unit_dictionary['tf2::StampedTransform'] = {'getOrigin': {'meter': 1.0}, 
                                                             'getRotation': {'quaternion': 1.0},
                                                             'stamp_': {'second': 1.0}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Transform>'] = {'getOrigin': {'meter': 1.0}, 
                                                                    'getRotation': {'quaternion': 1.0},
                                                                    'stamp_': {'second': 1.0}}

    # VISUALIZATION MSGS - http://wiki.ros.org/visualization_msgs
        # ADDED BECAUSE IT CONTAINS AN EMBEDDED POSE
        self.ros_unit_dictionary['visualization_msgs::Marker'] = {'position': {'meter': 1.0},  
                                                                  'orientation': {'quaternion': 1.0},
                                                                  'lifetime': {'second': 1.0},
                                                                  #'points': {'meter': 1.0},
                                                                  'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::MarkerArray'] = {'position': {'meter': 1.0},  
                                                                       'orientation': {'quaternion': 1.0},
                                                                       'lifetime': {'second': 1.0},
                                                                       #'points': {'meter': 1.0},
                                                                       'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::InteractiveMarker'] = {'position': {'meter': 1.0},  
                                                                             'orientation': {'quaternion': 1.0},
                                                                             'lifetime': {'second': 1.0},
                                                                             #'points': {'meter': 1.0},
                                                                             'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::InteractiveMarkerControl'] = {'position': {'meter': 1.0},  
                                                                                    'orientation': {'quaternion': 1.0},
                                                                                    'lifetime': {'second': 1.0},
                                                                                    #'points': {'meter': 1.0},
                                                                                    'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::InteractiveMarkerFeedback'] = {'position': {'meter': 1.0},  
                                                                                     'orientation': {'quaternion': 1.0},
                                                                                     #'mouse_point': {'meter': 1.0},
                                                                                     'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::InteractiveMarkerInit'] = {'position': {'meter': 1.0},  
                                                                                 'orientation': {'quaternion': 1.0},
                                                                                 'lifetime': {'second': 1.0},
                                                                                 #'points': {'meter': 1.0},
                                                                                 'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::InteractiveMarkerPose'] = {'position': {'meter': 1.0},  
                                                                                 'orientation': {'quaternion': 1.0},
                                                                                 'stamp': {'second': 1.0}}
        self.ros_unit_dictionary['visualization_msgs::InteractiveMarkerUpdate'] = {'position': {'meter': 1.0},  
                                                                                   'orientation': {'quaternion': 1.0},
                                                                                   'lifetime': {'second': 1.0},
                                                                                   #'points': {'meter': 1.0},
                                                                                   'stamp': {'second': 1.0}}


    # ROS TIME
        if not self.should_ignore_time_and_math:
            # ALL TIME UNITS ARE SECONDS REGARDLESS OF SCALE
            self.ros_unit_dictionary['ros::Time'] = {'sec': {'second': 1.0}, 
                                                     'nsec': {'second': 1.0}}  
            self.ros_unit_dictionary['ros::Duration'] = {'sec': {'second': 1.0}, 
                                                        'nsec': {'second': 1.0}}  
            self.ros_unit_dictionary['std::vector<ros::Duration>'] = {'sec': {'second': 1.0}, 
                                                                      'nsec': {'second': 1.0}}

            self.ros_unit_dictionary['ros::Rate'] = {'rate': {'second': -1.0}}


    # KNOWN FUNCTIONS
        if not self.should_ignore_time_and_math:
            self.ros_unit_dictionary['atan2'] = {'atan2': {'radian': 1.0}} 
            self.ros_unit_dictionary['acos'] = {'acos': {'radian': 1.0}} 
            self.ros_unit_dictionary['asin'] = {'asin': {'radian': 1.0}} 
            self.ros_unit_dictionary['atan'] = {'atan': {'radian': 1.0}} 
            self.ros_unit_dictionary['toSec'] = {'toSec': {'second': 1.0}}
            self.ros_unit_dictionary['toNSec'] = {'toNSec': {'second': 1.0}} 

        # self.ros_unit_dictionary['getX'] = {'getX': {'meter': 1.0}} 
        # self.ros_unit_dictionary['getY'] = {'getY': {'meter': 1.0}} 
        # self.ros_unit_dictionary['getZ'] = {'getZ': {'meter': 1.0}} 
        self.ros_unit_dictionary['quatToRPY'] = {'quatToRPY': {'radian': 1.0}}
        self.ros_unit_dictionary['getYaw'] =  {'getYaw': {'radian': 1.0}} 
        self.ros_unit_dictionary['getRoll'] =  {'getRoll': {'radian': 1.0}} 
        self.ros_unit_dictionary['getPitch'] =  {'getPitch': {'radian': 1.0}}

 
    # KNOWN SYMBOLS
        # self.ros_unit_dictionary['M_PI'] = {'M_PI': {'radian': 1.0}} 
        if self.should_use_dt_heuristic:
            self.ros_unit_dictionary['dt'] = {'dt': {'second': 1.0}}
        

    def convert_vector_units_to_dict(self, units_as_str):
        # STRIP EXTERIOR BRACKETS IF PRESENT
        units_as_str = units_as_str.replace('[', '').replace(']','')
        return_dict = {}
        units_as_list = units_as_str.split(',')
        # CHECK FOR ALL ZEROs
        if all([x=='0' for x in units_as_list]):
            # STRONG DIMENSIONLESS
            return self.ros_unit_dictionary['dimensionless']
        return_dict['meter'] = float(units_as_list[0])
        return_dict['second'] = float(units_as_list[1])
        return_dict['radian'] = float(units_as_list[2])
        return_dict['degree_360'] = float(units_as_list[3])
        return_dict['quaternion'] = float(units_as_list[4])
        return_dict['kilogram'] = float(units_as_list[5])
        return_dict['amp'] = float(units_as_list[6])
        return_dict['degree_celsius'] = float(units_as_list[7])
        return_dict['mol'] = float(units_as_list[8])
        return_dict['candela'] = float(units_as_list[9])
        # FILTER ZEROS
        return_dict = {k: v for k, v in return_dict.iteritems() if v != 0.0}
        return return_dict


if __name__ == '__main__':
    sh = SymbolHelper()


