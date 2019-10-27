#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import logging
from typing import Any, List

from nltk import ParentedTree

from nlp_crow.modules.CrowModule import CrowModule
import nltk

from nlp_crow.structures.tagging.MorphCategory import POS
from nlp_crow.structures.tagging.ParsedText import ParsedText, TaggedText, ParseTreeNode, TaggedToken
from nlp_crow.structures.tagging.Tag import Tag

#from scripts.test_grammar import get_parse_tree

class NLTK:
    def tag(self, text : str) -> TaggedText:
        """
        Tags a text.

        Parameters
        ----------
        text    an input text as string

        Returns
        -------
        a tagged text object
        """
        tagged_text =  TaggedText()
        tokens = nltk.word_tokenize(text)

        for pair in nltk.pos_tag(tokens):
            tag = Tag()
            tag.pos = POS(value=pair[1])
            tagged_text.add_tagged_token(token=pair[0], tag=tag)

        return tagged_text


class GrammarParser(CrowModule):
    """
    Tags and parses the text
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.nltk_tagger = NLTK()


    def parse(self, sentence : str) -> ParsedText:
        """
        Currently used for dummy text parsing. After the text is tagged, it is split on "and" and "." tokens
        into sentences. Each sentence has its tokens hanged under an "S" node.
        TODO: swap with the parse() method which relies on a grammar

        Parameters
        ----------
        sentence    an input sentence as a string

        Returns
        -------
        parsed text
        """

        # use NLTK for tagging
        tagged_text = self.nltk_tagger.tag(sentence)

        # create a new object for parsed text
        parsed_text = ParsedText()

        # save the original text
        parsed_text.orig_text = sentence

        # create the root of the tree
        root = ParseTreeNode(label="T")
        parsed_text.parse_tree = root

        # create a parent node for the first sentence
        sentence_node = ParseTreeNode(label="S")

        # sequentially process the tagged tokens
        for tagged_token in tagged_text.get_tokens_with_tags():
            if tagged_token.token in ["and", "."]:
                # in case there is a previous sentence
                if sentence_node.subnodes:
                    # append the previous sentence node under the root node
                    root.subnodes.append(sentence_node)
                    # and start a new sentence
                    sentence_node = ParseTreeNode(label="S")

                # append the separating token under the root node
                root.subnodes.append(tagged_token)
            else:
                # append the token to the current sentence
                sentence_node.subnodes.append(tagged_token)

        if sentence_node.subnodes:
            # finalize the last sentence
            root.subnodes.append(sentence_node)

        self.logger.debug(f"Parsed text: {parsed_text}")

        return parsed_text

    # TODO this method should be used in the future, relies on a grammar
    # def parse(self, sentence : str) -> ParsedText:
    #     tree = get_parse_tree(sentence)
    #
    #     tokens = nltk.word_tokenize(sentence)
    #
    #     root, _ = self.transform_recursive(tree, tokens)
    #
    #     parsed_text = ParsedText()
    #     parsed_text.orig_text = sentence
    #     parsed_text.parse_tree = root
    #
    #     self.logger.debug(f"Parsed text: {parsed_text}")
    #
    #     return parsed_text


    def transform_recursive(self, node : Any, tokens : List):
        """
        Recursively transforms the tree from the format of the grammar parser to the format used in the NL processing.

        Parameters
        ----------
        node    a node to be processed - can be either a ParentedTree object or a string
                (for the first call this should be the tree root)
        tokens  a list of tokens (not provided in the tree from the grammar parser)

        Returns
        -------
        the recursively transformed node, the list of remaining tokens
        """
        if type(node) == ParentedTree:
            return self.transform_node(node, tokens)

        elif type(node) == str:
            return self.transform_tag(node, tokens[0]), tokens[1:]


    def transform_node(self, node, tokens):
        """
        Transforms a node by recursively calling transform_recursive() on its subnodes.
        """
        label = node._label
        parse_tree_node = ParseTreeNode(label=label)

        for subnode in node:
            parse_tree_subnode, tokens = self.transform_recursive(subnode, tokens)
            parse_tree_node.subnodes.append(parse_tree_subnode)

        return parse_tree_node, tokens


    def transform_tag(self, node, token):
        """
        Transforms a single token and its tag (in the string form) into a tagged token.
        """
        tagged_token = TaggedToken()

        tagged_token.token = token
        tagged_token.tag = Tag(pos=POS(node))

        return tagged_token