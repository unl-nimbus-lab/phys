#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import operator
from str_utils import *
from time import gmtime, strftime
import os
import pickle


VAR_SIM_FACTOR = 0.0
SUFFIX_SIM_FACTOR = 1.0

def _log(msg):
    print (msg)


class TypeMiner(object):
    def __init__(self, training_file, types_file, suffix_file):
        self.training_file = training_file
        self.types_file = types_file
        self.suffix_file = suffix_file

        self.vars = []
        self.var2type = {}
        self.suffix2type = {}


    def train(self, should_reuse_training=False):
        _log("starting init raw data ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
        # VARIABLES FOR REUSING 'TRAINING'
        self_vars_pkl_filename = './datamining_self_vars.pkl'
        self_var2type_pkl_filename = './datamining_self_var2type.pkl'
        if should_reuse_training and (
            # TRY TO RE-USE PKL FILES
            os.path.exists(self_vars_pkl_filename) and
            os.path.exists(self_var2type_pkl_filename)):
                # LOAD FILES
                self.vars = pickle.load(open(self_vars_pkl_filename, 'rb'))
                self.var2type = pickle.load(open(self_var2type_pkl_filename, 'rb'))
        else:
            # NO PRE-EXISTING DATA - RECREATE / RE-LOAD
            self._init_raw_data()
            if should_reuse_training:
                # SAVE CURRENT DATA FOR POSSIBLE REUSE
                pickle.dump(self.vars, open(self_vars_pkl_filename, 'wb'))
                pickle.dump(self.var2type, open(self_var2type_pkl_filename, 'wb'))

        _log("starting init suffix data... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
        self._init_suffix_data()
        _log("ending init raw data ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))


    def _init_raw_data(self):
        # VARIABLE NAME AND TYPE TRAINING DATA
        # LOAD TRAINING DATA FROM FILE
        with open(self.training_file) as f:
            # https://stackoverflow.com/questions/19062574/python-read-file-into-list-strip-newlines
            # raw_training_data_as_list = [line.rstrip('\n') for line in f]
            for training_example in (line.rstrip('\n') for line in f):
                var_name, unit_type = training_example.split(',', 1)
                var_name, unit_type = var_name.strip(), unit_type.strip()
                useful_term = self._get_meaningful_term(var_name)
                if useful_term:
                    self.vars.append(useful_term)
                    try:
                        self.var2type[useful_term][unit_type] += 1
                    except KeyError:
                        try:
                            self.var2type[useful_term][unit_type] = 1
                        except KeyError:
                            self.var2type[useful_term] = {unit_type: 1}
        print ('End... %d %d' % (len(self.vars), len(self.var2type)))

    def _init_suffix_data(self):
        # LOAD VARIABLE SUFFIX AND TYPE DATA
        with open(self.suffix_file) as f:
            for suffix_item in (line.rstrip('\n') for line in f):
                suffix, unit_type = suffix_item.split(',', 1)
                suffix, unit_type = suffix.strip(), unit_type.strip()
                self.suffix2type[suffix] = unit_type


    def predict_proba(self, vname):
        term = self._get_meaningful_term(vname)
        #print term
        if not term:
            return {}

        type2maxsim = {}

        _extract_str_distance = self._extract_str_distance
        var2sim = {v: (1.0 - _extract_str_distance(term, v)) for v in self.vars}
        var_sims = sorted(var2sim.iteritems(), key=operator.itemgetter(1), reverse=True)
        for var, sim in var_sims:
            types = sorted(self.var2type[var].iteritems(),
                           key=operator.itemgetter(1), reverse=True)
            for t in types:
                t = t[0]
                # type_sims.append((var, t))
                # type_sims.append((t[0], '%.4f' % sim))
                if t not in type2maxsim:
                    type2maxsim[t] = {'sim1': sim, 'sim2': 0.0}
                elif sim > type2maxsim[t]['sim1']:
                    type2maxsim[t]['sim1'] = sim
        
        _extract_suffix_distance = self._extract_suffix_distance
        suffix2sim = {s: (1.0 - _extract_suffix_distance(term, s)) for s in self.suffix2type}
        suffix_sims = sorted(suffix2sim.iteritems(), key=operator.itemgetter(1), reverse=True)
        for suffix, sim in suffix_sims:
            t = self.suffix2type[suffix]
            if t not in type2maxsim:
                type2maxsim[t] = {'sim1': 0.0, 'sim2': sim}
            elif sim > type2maxsim[t]['sim2']:
                type2maxsim[t]['sim2'] = sim

        type2sim = {}
        for t, sim_dict in type2maxsim.items():
            type2sim[t] = VAR_SIM_FACTOR * sim_dict['sim1'] + SUFFIX_SIM_FACTOR * sim_dict['sim2']

        type2prob = {}
        for t, sim in type2sim.items():
            type2prob[t] = 0.5 + (sim/2.0)

        return type2prob


    def _get_meaningful_term(self, var_name):
        terms = self._split_var(var_name)
        terms = filter(lambda x: len(x) > 1, terms)
        if not terms:
            return ''
        else:
            terms = self._get_nouns(terms)
            if terms:
                #return terms[-1]
                if len(terms) > 1:
                    return terms[-2]+terms[-1]
                else:
                    return terms[-1]
            return ''


    def _split_var(self, name):
        final_names = []
        # try to remove tail digits first
        names = map(self._remove_tail_digits, re.split(r'[_|\.]', name))
        for name in names:
            if self._check_if_all_capitals(name):
                final_names.append(name.lower())
            else:
                subnames = self._split_by_captials(name)
                subnames = map(self._remove_tail_digits, subnames)
                subnames = map(str.lower, subnames)
                final_names.extend(subnames)
        final_names = filter(bool, final_names)
        return final_names


    @staticmethod
    def _remove_tail_digits(name):
        return name.rstrip('1234567890')


    @staticmethod
    def _check_if_all_capitals(name):
        return name.upper() == name


    @staticmethod
    def _split_by_captials(name):
        return filter(bool, re.split('([A-Z][^A-Z]*)', name))


    @staticmethod
    def _split_by_dot(name):
        names = re.split(r'[.]', name)
        return names


    def _get_nouns(self, names):
        #nouns = [str(w) for w, t in get_pos(' '.join(names)) if t==wn.NOUN]
        nouns = []
        for name in names:
            noun = [str(w) for w, t in get_pos(name) if t==wn.NOUN]
            if noun:
                nouns.extend(noun)
        return nouns


    def _extract_str_distance(self, name1, name2):
        dis = (self._compute_substr_distance(name1, name2)
               + self._compute_abbrvstr_distance(name1, name2)) / 2
        return dis


    @staticmethod
    def _compute_substr_distance(name1, name2):
        min_len = min(len(name1), len(name2))
        substr = list(find_longest_common_str(name1, name2))
        return 1 - (len(substr[0]) if substr else 0) * 1.0 / min_len


    @staticmethod
    def _compute_abbrvstr_distance(name1, name2):
        if len(name1) > len(name2):
            name1, name2 = (name2, name1)
        return 1 - is_abbrev_for_multiple(name1, [name2])

   
    def _extract_suffix_distance(self, name, suffix):
        dis = self._compute_suffix_distance(name, suffix)
        return dis

    
    @staticmethod
    def _compute_suffix_distance(name, suffix):
        substr = list(find_longest_common_str(name, suffix))
        return 1 - (len(substr[0]) if substr else 0) * 1.0 / len(suffix)


    @staticmethod
    def _compute_levenshtein_distance(name, suffix):
        return get_levenshtein_dist(name, suffix)


    @staticmethod
    def _compute_jarowinkler_distance(name, suffix):
        return 1.0 - get_jarowinkler_dist(name, suffix)


