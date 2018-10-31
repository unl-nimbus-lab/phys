#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import operator
from str_utils import *
from time import gmtime, strftime
import os


VAR_SIM_FACTOR = 0.0
SUFFIX_SIM_FACTOR = 1.0

def _log(msg):
    print (msg)


class TypeMiner(object):
    def __init__(self, training_file, types_file, suffix_file):
        self.suffix_file = suffix_file
        self.suffix2type = {}
        self.excluded_nouns = ['factor', 'threshold', 'vector', 'controller', 'range', 'motor', \
                               'granularity', 'trajectory', 'platform', \
                               'offset', 'constant', 'ratio', 'scale', \
                               'val', 'value', 'param', 'params', 'parameter', 'parameters', \
                               'state', 'delta', 'gain', 'data', 'max', 'min', 'ref']

        #offset, constant, ratio, scale, val, value, param, params, parameter, parameters, 
        #state, delta, gain, data, max, min, ref

        self.included_nouns = ['acc']

        #acc, pose


    def train(self, should_reuse_training=False):
        _log("starting init suffix data... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
        self._init_suffix_data()
        _log("ending init suffix data ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))


    def _init_suffix_data(self):
        # LOAD VARIABLE SUFFIX AND TYPE DATA
        with open(self.suffix_file) as f:
            for suffix_item in (line.rstrip('\n') for line in f):
                suffix, unit_type = suffix_item.split(',', 1)
                suffix, unit_type = suffix.strip(), unit_type.strip()
                self.suffix2type[suffix] = unit_type


    def predict_proba(self, vname):
        terms = self._get_meaningful_term(vname)
        #print terms
        if not terms:
            return {}

        type2maxsim = {}

        i = 1
        n = len(terms)
        if n > 2: n = 2
        while i<=n:
            term = terms[-i]

            _extract_suffix_distance = self._extract_suffix_distance
            suffix2sim = {s: (1.0 - _extract_suffix_distance(term, s)) for s in self.suffix2type}
            suffix_sims = sorted(suffix2sim.iteritems(), key=operator.itemgetter(1), reverse=True)
            for suffix, sim in suffix_sims:
                t = self.suffix2type[suffix]
                if t not in type2maxsim:
                    type2maxsim[t] = sim
                elif sim > type2maxsim[t]:
                    type2maxsim[t] = sim

            i+=1

        type2prob = {}
        for t, sim in type2maxsim.items():
            type2prob[t] = 0.5 + (sim/2.0)

        return type2prob


    def _get_meaningful_term(self, var_name):
        terms = self._split_var(var_name)
        terms = filter(lambda x: len(x) > 1, terms)
        terms = filter(lambda x: x not in self.excluded_nouns, terms)
        if not terms:
            return ''
        else:
            terms = self._get_nouns(terms)
            if terms:
                return terms
            return ''


    def predict_proba_2(self, vname):
        term = self._get_meaningful_term_2(vname)
        #print term
        if not term:
            return {}

        type2maxsim = {}
        
        _extract_suffix_distance = self._extract_suffix_distance
        suffix2sim = {s: (1.0 - _extract_suffix_distance(term, s)) for s in self.suffix2type}
        suffix_sims = sorted(suffix2sim.iteritems(), key=operator.itemgetter(1), reverse=True)
        for suffix, sim in suffix_sims:
            t = self.suffix2type[suffix]
            if t not in type2maxsim:
                type2maxsim[t] = sim
            elif sim > type2maxsim[t]:
                type2maxsim[t] = sim

        type2prob = {}
        for t, sim in type2maxsim.items():
            type2prob[t] = 0.5 + (sim/2.0)

        return type2prob


    def _get_meaningful_term_2(self, var_name):
        terms = self._split_var(var_name)
        terms = filter(lambda x: len(x) > 1, terms)
        terms = filter(lambda x: x not in self.excluded_nouns, terms)
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
        names = map(self._remove_tail_digits, re.split(r'[_|\.]', name))
        for name in names:
            name = self._remove_array_index(name)
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
    def _remove_array_index(name):
        return re.sub(r'\[.*\]', '', name)


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
            elif name in self.included_nouns:
                nouns.append(name)
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
        #dis = self._compute_suffix_distance(name, suffix)
        dis = self._compute_suffix_start_distance(name, suffix)
        #dis = self._compute_suffix_start_distance_2(name, suffix)
        return dis

    
    @staticmethod
    def _compute_suffix_start_distance(name, suffix):
        substr = list(find_longest_common_str(name, suffix))

        #TODO terms matched at most 2
        myss = None
        for ss in substr:
            #if ss[:1] == suffix[:1] and ss[:1] == name[:1] and len(ss) >= 3:
            #if ss[:1] == suffix[:1] and len(ss) >= 3:
            #if ss[:1] == suffix[:1]:
            if ss[:3] == suffix[:3] and len(ss) >= 3:
                myss = ss
                break

        return 1 - (len(myss) if myss else 0) * 1.0 / len(suffix)


    @staticmethod
    def _compute_suffix_start_distance_2(name, suffix):
        substr = list(find_longest_common_str(name, suffix))

        myss = None
        for ss in substr:
            if ss[:3] == suffix[:3] and len(ss) >= 3:
                myss = ss
                break

        pos_factor = 0.0
        if myss:
            p = name.find(myss)
            l = len(name)
            pos_factor = (l-p)*1.0/l

        return 1 - (len(myss) if myss else 0) * 1.0 * pos_factor / len(suffix)

    
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


