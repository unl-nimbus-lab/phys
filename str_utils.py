#!/usr/bin/python
# _*_ coding:utf-8 _*_

__author__ = 'root'

import re
from nltk.corpus import wordnet as wn
from pattern.en import tag, singularize
from distance import nlevenshtein
from pyjarowinkler import distance


def find_longest_common_str(S, T):
    m = len(S)
    n = len(T)
    counter = [[0]*(n+1) for x in range(m+1)]
    longest = 0
    lcs_set = set()
    for i in range(m):
        for j in range(n):
            if S[i] == T[j]:
                c = counter[i][j] + 1
                counter[i+1][j+1] = c
                if c > longest:
                    lcs_set = set()
                    longest = c
                    lcs_set.add(S[i-c+1:i+1])
                elif c == longest:
                    lcs_set.add(S[i-c+1:i+1])

    return lcs_set


def is_abbrev_for_single(word1, word2):
    if not word1:
        return True
    if not word2:
        return False
    if word1[0] != word2[0]:
        return False
    len_1, len_2 = len(word1), len(word2)
    if len_1 > len_2:
        return False
    levdis = levenshtein(word1, word2)
    return levdis == (len_2 - len_1), (len_2 - levdis)


def _max_common_abbrev_len(abbrev, word, depth=0):
    if not (abbrev and word):
        return depth
    if abbrev[0] == word[0]:
        return _max_common_abbrev_len(abbrev[1:], word[1:], depth + 1)
    elif depth == 0:
        return 0
    else:
        return _max_common_abbrev_len(abbrev, word[1:], depth)


def is_abbrev_for_multiple(abbrev, words):
    if not words:
        return False
    max_len = _max_common_abbrev_len(abbrev, words[0])
    if len(abbrev) == max_len:
        return True
    return is_abbrev_for_multiple(abbrev[max_len:], words[1:])



def get_pos(text):
    return [(w, penn_to_wn(p)) for w, p in tag(text)]


def is_singular(text):
    return singularize(text) is not text


def is_noun(tag):
    return tag in ['NN', 'NNS', 'NNP', 'NNPS']


def is_verb(tag):
    return tag in ['VB', 'VBD', 'VBG', 'VBN', 'VBP', 'VBZ']


def is_adverb(tag):
    return tag in ['RB', 'RBR', 'RBS']


def is_adjective(tag):
    return tag in ['JJ', 'JJR', 'JJS']


def penn_to_wn(tag):
    if is_adjective(tag):
        return wn.ADJ
    elif is_noun(tag):
        return wn.NOUN
    elif is_adverb(tag):
        return wn.ADV
    elif is_verb(tag):
        return wn.VERB
    return None


def get_levenshtein_dist(word1, word2):
    return nlevenshtein(word1, word2, method=1)


def get_jarowinkler_dist(word1, word2):
    return distance.get_jaro_distance(word1, word2, winkler=True, scaling=0.1)



if __name__ == '__main__':
    print is_abbrev_for_multiple('multi', ['immutable', 'multi', 'dict'])
