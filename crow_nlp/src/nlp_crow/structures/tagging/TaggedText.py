#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from typing import List, Tuple

from nlp_crow.database.Database import Database
from nlp_crow.structures.tagging.MorphCategory import POS
from nlp_crow.structures.tagging.Tag import Tag

import owlready2 as ow
import re

db = Database()

# with db.onto as onto:
class TaggedToken(ow.Thing):
    namespace = db.onto
    def __repr__(self):
        return f"\"{self.token}\"/{self.tag}"

class TaggedText(ow.Thing):
    """
    A sequence of tokens and their respective tags.
    """
    namespace = db.onto
    def add_tagged_token(self, token : str, tag : Tag):
        """
        Add a token and its tag.

        Parameters
        ----------
        token  a token to be added
        tag    a tag for the token
        """
        self.tokens.append(token)
        self.tags.append(tag)

    def get_tokens_with_tags(self):
        """
        A generator which yields the sequence of tagged tokens.
        """
        for token, tag in zip(self.tokens, self.tags):
            yield TaggedToken(token=token, tag=tag)

    def get_text(self) -> str:
        """
        Returns the tokens as a space-separated plain text.
        """
        return " ".join(self.tokens)

    def contains_pos_token(self, token : str, pos_str : str, case_sensitive=False, include_subcategories : bool = True):
        """
        See contains_tagged_token(). This is syntax sugar allowing the POS tag to be given as a string (e.g. "NN").
        """
        return self.contains_tagged_token(token=token, tag=Tag(pos=POS(pos_str)),
                                          case_sensitive=case_sensitive, include_subcategories=include_subcategories)

    def contains_tagged_token(self, token : str, tag : Tag, case_sensitive : bool = False, include_subcategories : bool = True):
        """
        Returns True if the tagged text contains a token with the corresponding tag, False otherwise.

        Parameters
        ----------
        token                   the token as a string
        tag                     the tag as a Tag object
        case_sensitive          if the token matching should be case-sensitive
        include_subcategories   if POS subcategories should be considered as matching (e.g. "NN" tag
                                will be matched for "N" query)
        """
        for a, b in zip(self.tokens, self.tags):
            if self.token_equals(a, token, case_sensitive) and self.tag_equals(b, tag, include_subcategories):
                return True
        return False

    def contains_text(self, text : str, case_sensitive : bool = False):
        """
        Returns True if the text contains the given plain-text pattern, False otherwise.

        Parameters
        ----------
        text            pattern to be found in the text
        case_sensitive  if the matching should be case-sensitive
        """
        if case_sensitive:
            return text in self.get_text()

        return text.lower() in self.get_text().lower()

    def indices_of(self, token : str, tag_str : str = None, case_sensitive : bool = False, include_subcategories : bool = True):
        """
        Returns the all indices of the token in the tagged text or an empty list if the token is not present at all.

        If the tag_str is specified, the corresponding tags are checked and only the indices of matching pairs
        (token,tag) are returned in the list.


        Parameters
        ----------
        token           the token to be found in the text
        tag_str         the optional tag which has to match each token occurrence
        case_sensitive          if the token matching should be case-sensitive
        include_subcategories   if POS subcategories should be considered as matching (e.g. "NN" tag
                                will be matched for "N" query)
        """
        indices = [i for i, t in enumerate(self.tokens) if self.token_equals(t, token, case_sensitive)]

        if tag_str:
            indices = [i for i in indices if self.tag_equals(self.tags[i], Tag(pos=POS(tag_str)), include_subcategories)]

        return indices

    def match_regex(self, regex : str):
        """
        Return True if the text contains the given regex pattern, False otherwise.

        Parameters
        ----------
        text            regex pattern to be found in the text
        case_sensitive  if the matching should be case-sensitive
        """
        text = self.get_text()
        match = re.search(regex, text)

        return match

    def match_regex_pos_list(self, regex_pos_list : List[Tuple[str,str]]):
        """
        Return True if the text contains the given sequence of tagged tokens (tokens can be specified as regexes).
        The sequence is matched sequentially, i.e. the tagged tokens should be in the specified order.

        Parameters
        ----------
        regex_pos_list  a list of tuples (<regex_as_string>, <pos_as_string>), e.g. [(r"\w*", "NN"), (r".*", "CD")]
        """
        i = 0
        matches = []

        for tagged_token in self.get_tokens_with_tags():
            regex_to_match = regex_pos_list[i][0]
            pos_to_match = regex_pos_list[i][1]
            tag_to_match = Tag(pos=POS(pos_to_match))

            match = re.match(regex_to_match, tagged_token.token)

            if match and tagged_token.tag.equals(tag_to_match):
                matches.append(match)
                i += 1

        # all items were matched
        if i == len(regex_pos_list):
            return matches

        return None


    def cut(self, start_idx, end_idx):
        """
        Returns a new TaggedText object which is a slice of the original tagged text. Uses the pythonic way
        of slicing lists.

        Parameters
        ----------
        start_idx   the start index (inclusive)
        end_idx     the end index (exclusive)
        """
        return TaggedText(tokens=self.tokens[start_idx:end_idx], tags=self.tags[start_idx:end_idx])

    def token_equals(self, tok1 : str, tok2 : str, case_sensitive : bool):
        if not case_sensitive:
            tok1 = tok1.lower()
            tok2 = tok2.lower()

        return tok1 == tok2

    def tag_equals(self, tag1 : Tag, tag2 : Tag, include_pos_subcategories: bool):
        return tag1.equals(tag2, include_pos_subcategories)

    def __str__(self):
        return "TaggedText({})".format(", ".join([f"\"{tok}\"/{tag}"
                                                     for tok, tag in zip(self.tokens, self.tags)]))

    def __iadd__(self, other):
        for a, b in zip(other.tokens, other.tags):
            self.add_tagged_token(a, b)

        return self


# TODO check if all of these classes are saved in ontology and delete the code
class hasToken(TaggedToken >> str, ow.FunctionalProperty):
    namespace = db.onto
    python_name = "token"

class hasTag(TaggedToken >> Tag, ow.FunctionalProperty):
    namespace = db.onto
    python_name = "tag"

class hasTokens(TaggedText >> str):
    namespace = db.onto
    python_name = "tokens"

class hasTags(TaggedText >> Tag):
    namespace = db.onto
    python_name = "tags"