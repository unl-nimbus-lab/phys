#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pgm import Variable, Factor, FactorGraph, PGMEngine


class PGMPlayer(object):

    def __init__(self, fg_filename='test.fg'):
        self.fg_filename = fg_filename
        self.curr_factors = []
        self.strvar2pgmvar = {}
        Variable.reset()

    def add_factor(self, left, right, states, proba, comment):
        left = [self.get_var(x) for x in left]
        right = [self.get_var(x) for x in right]
        factor = Factor(vars=left + right,
                        states=map(lambda x: proba if x else 1 - proba, states),
                        comment=comment)
        self.curr_factors.append(factor)

    def _build_factor_graph(self):
        factor_graph = FactorGraph()
        for c in self.curr_factors:
            factor_graph.add_factor(c)
        return factor_graph

    def compute_marginals(self, alg='BP'):
        factor_graph = self._build_factor_graph()
        pgmengine = PGMEngine(factor_graph)
        pgmengine.prepare(self.fg_filename, alg)
        pgmengine.run()
        pgmvar2proba = pgmengine.query_all_var_marginals()
        return {pv: p0 for pv, (p0, _) in pgmvar2proba.iteritems()}

    def get_var(self, vname):
        try:
            var = self.strvar2pgmvar[vname]
        except KeyError:
            var = Variable(vname, id_=None, nstates=2)
            self.strvar2pgmvar[vname] = var
        return var


if __name__ == '__main__':
    
    player = PGMPlayer()
    player.add_factor(left=[], right=['n1'],
                      states=[0, 1],
                      proba=1.0,
                      comment='n1 = 1')
    player.add_factor(left=['n1'], right=['p1'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n1 -> p1')
    player.add_factor(left=[], right=['n2'],
                      states=[0, 1],
                      proba=1.0,
                      comment='n2 = 1')
    player.add_factor(left=['n2'], right=['p2'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n2 -> p2')
    player.add_factor(left=[], right=['n3'],
                      states=[0, 1],
                      proba=1.0,
                      comment='n3 = 1')
    player.add_factor(left=['n3'], right=['p3'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n3 -> p3')
    player.add_factor(left=[], right=['n4'],
                      states=[0, 1],
                      proba=1.0,
                      comment='n4 = 1')
    player.add_factor(left=['n4'], right=['p4'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n4 -> p4')
    player.add_factor(left=[], right=['n5'],
                      states=[0, 1],
                      proba=1.0,
                      comment='n5 = 1')
    player.add_factor(left=['n5'], right=['p5'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n5 -> p5')
    player.add_factor(left=[], right=['n6'],
                      states=[0, 1],
                      proba=1.0,
                      comment='n6 = 1')
    player.add_factor(left=['n6'], right=['p6'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n6 -> p6')
    player.add_factor(left=[], right=['n7'],
                      states=[0, 1],
                      proba=0.6499,
                      comment='n7 = 1')
    player.add_factor(left=['n7'], right=['p7'],
                      states=[1, 0, 1, 1],
                      proba=0.7,
                      comment='n7 -> p7')
    player.add_factor(left=[], right=['c3'],
                      states=[0, 1],
                      proba=1.0,
                      comment='c3 = 1')
    player.add_factor(left=['c3'], right=['p3'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='c3 -> p3')
    player.add_factor(left=[], right=['c4'],
                      states=[0, 1],
                      proba=1.0,
                      comment='c4 = 1')
    player.add_factor(left=['c4'], right=['p4'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='c4 -> p4')
    player.add_factor(left=['p3'], right=['p1'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p3 -> p1')
    player.add_factor(left=['p1'], right=['p3'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p1 -> p3')
    player.add_factor(left=['p3'], right=['p8'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p3 -> p8')
    player.add_factor(left=['p8'], right=['p3'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p8 -> p3')
    player.add_factor(left=['p4'], right=['p2'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p4 -> p2')
    player.add_factor(left=['p2'], right=['p4'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p2 -> p4')
    player.add_factor(left=['p5'], right=['p6'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p5 -> p6')
    player.add_factor(left=['p6'], right=['p5'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p6 -> p5')
    player.add_factor(left=[], right=['p5'],
                      states=[0, 1],
                      proba=0.95,
                      comment='p5 = 1')
    player.add_factor(left=[], right=['p9'],
                      states=[0, 1],
                      proba=0.95,
                      comment='p9 = 1')
    player.add_factor(left=['p6'], right=['p7'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p6 -> p7')
    player.add_factor(left=['p7'], right=['p6'],
                      states=[1, 0, 1, 1],
                      proba=0.95,
                      comment='p7 -> p6')
     
    print {v.name: '%.4f' % (1.0 - p) for v, p in player.compute_marginals().iteritems()}


    player = PGMPlayer()
    player.add_factor(left=['a', 's'], right='y',
                      states=[1, 1, 1, 0, 1, 1, 1, 1],
                      proba=0.95,
                      comment='a & s -> y')
    player.add_factor(left=['s', 'y'], right=['a'],
                      states=[1, 1, 1, 0, 1, 1, 1, 1],
                      proba=0.95,
                      comment='s & y -> a')
    player.add_factor(left=['a', 'y'], right=['s'],
                      states=[1, 1, 1, 0, 1, 1, 1, 1],
                      proba=0.95,
                      comment='a & y -> s')
    player.add_factor(left=[], right=['a'],
                      states=[0, 1],
                      proba=0.05,
                      comment='a = 0.5')
    player.add_factor(left=[], right=['y'],
                      states=[0, 1],
                      proba=0.95,
                      comment='y = 0.1')
    player.add_factor(left=[], right=['s'],
                      states=[0, 1],
                      proba=0.95,
                      comment='s = 0.5')
    print {v.name: '%.4f' % (1.0 - p) for v, p in player.compute_marginals().iteritems()}


