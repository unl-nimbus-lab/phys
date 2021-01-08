#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Created by Zhaogui Xu on 8/12/16

from os.path import join, dirname
from StringIO import StringIO
import dai


class Variable(object):

    name2id = {}
    count = 0

    def __init__(self, name, id_=None, nstates=2):
        self.id = self.get_id(name) if id_ is None else id_
        self.name = name
        self.nstates = nstates

    @classmethod
    def get_id(cls, name):
        try:
            return cls.name2id[name]
        except KeyError:
            id_ = cls.count
            cls.name2id[name] = id_
            cls.count += 1
        return id_

    @classmethod
    def reset(cls):
        cls.name2id.clear()
        cls.count = 0

    def __str__(self):
        return 'x%d: %s(%d)' % (self.id, self.name, self.nstates)

    def __hash__(self):
        return self.id

    def __eq__(self, other):
        return isinstance(other, Variable) and self.id == other.id


class Factor(object):
    def __init__(self, vars, states, comment):
        self.vars = vars
        self.states = states
        self.comment = comment
        assert (2 ** len(self.vars)) == len(self.states), \
            'unvalid factor (%d, %d): %s' % (len(self.vars), len(self.states),
                                             self.comment)

    def __str__(self):
        vars_str = '\n'.join(str(v) for v in self.vars)
        states_str = 'states: [' + ', '.join(str(s) for s in self.states) + ']'
        return self.comment + '\n' + vars_str + '\n' + states_str


class FactorGraph(object):
    def __init__(self, vars=(), factors=()):
        self.vars = vars or set()
        self.factors = factors or []

    def add_factor(self, factor):
        self.factors.append(factor)
        self.vars.update(factor.vars)

    def __str__(self):
        return '\n'.join(self.factors)

    def dump(self, filename):
        with open(filename, 'w') as fp:
            fp.write(self.dumps())
            fp.flush()

    def dumps(self):
        buf = StringIO()
        buf.write('# number of nodes\n')
        buf.write('# labels of nodes\n')
        buf.write('# states of nodes\n')
        buf.write('# number of factors\n')
        buf.write('# index \t value\n')
        buf.write(str(len(self.factors)))
        buf.write('\n')
        for factor in self.factors:
            #buf.write('\n'.join(('# x%d: %s(%d)' % (v.id, v.name, v.nstates)) for v in factor.vars))
            buf.write('# ' + str(factor).replace('\n', '\n# '))
            buf.write('\n')
            buf.write(str(len(factor.vars)))
            buf.write('\n')
            buf.write(' '.join(str(v.id) for v in factor.vars))
            buf.write('\n')
            buf.write(' '.join(str(v.nstates) for v in factor.vars))
            buf.write('\n')
            buf.write(str(len(factor.states)))
            buf.write('\n')
            buf.write('\n'.join(('%d\t%s' % (i, s)) for i, s in enumerate(factor.states)))
            buf.write('\n\n')
        return buf.getvalue()


class PGMEngine(object):
    def __init__(self, factor_graph):
        self.factor_graph = factor_graph
        self.method_aliases = None
        self.method2inference = {}
        self.method2props = {}
        self.method = ''
        self.dai_factor_graph = None
        self.inference = None

    def prepare(self, fg_filename, method):
        self._prepare_dai_factor_graph(fg_filename)
        self._prepare_method_aliases(join(dirname(__file__), 'aliases.conf'))
        self.load_inference(method)

    def _prepare_dai_factor_graph(self, filename):
        self.factor_graph.dump(filename)
        self.dai_factor_graph = dai.FactorGraph()
        self.dai_factor_graph.ReadFromFile(filename)

    def _prepare_method_aliases(self, filename):
        self.method_aliases = dai.readAliasesFile(filename)

    def load_inference(self, method):
        if method in self.method2inference:
            self.method = method
            self.inference = self.method2inference[method]
        else:
            mthdname2props = dai.parseNameProperties(method, self.method_aliases)
            alg = dai.newInfAlg(mthdname2props.first, self.dai_factor_graph,
                                mthdname2props.second)
            self.method2props[method] = mthdname2props
            self.method2inference[method] = alg
            self.method = method
            self.inference = alg

    def run(self):
        self.inference.init()
        self.inference.run()

    @property
    def vars(self):
        return self.factor_graph.vars

    @property
    def factors(self):
        return self.factor_graph.factors

    def query_var_marginal(self, var):
        # after test, we find the id is exactly equal to the index
        # TODO: may refactor it in the future
        factor = self.inference.beliefV(var.id)
        return factor[0], factor[1]

    def query_all_var_marginals(self):
        return {var: self.query_var_marginal(var)
                for var in self.factor_graph.vars}

    # def query_factor_marginal(self, factor):
    #     #TODO:
    #     i = self.factors.index(factor)
    #     factor = self.inference.beliefF(i)
    #     return tuple([int(str(e)[1:]) for e in factor.vars().elements()]), \
    #         tuple([factor[i] for i in range(factor.nrStates())])
    #
    # def query_all_factor_marginals(self):
    #     return [self.query_factor_marginal(i) for i in range(self.factor_graph.nrFactors())]

    def query_map(self):
        return map(int, self.inference.findMaximum())
