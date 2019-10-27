#!/usr/bin/env python
"""
Copyright (c) 2019 Zdenek Kasner
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from nlp_crow.modules.CrowModule import CrowModule
from nlp_crow.structures.tagging.MorphCategory import POS
from nlp_crow.structures.tagging.Tag import Tag
from nlp_crow.structures.tagging.TaggedText import TextParse
from nlp_crow.utils.files import get_full_path
import ufal.morphodita as morphodita


class Tagger(CrowModule):
    def __init__(self):
        # self.nlp_model_en_path = get_full_path("morphodita", "english-morphium-wsj-140407", "english-morphium-wsj-140407.tagger")
        self.nlp_model_cs_path = get_full_path("morphodita", "czech-morfflex-pdt-161115", "czech-morfflex-pdt-161115.tagger")

        self.tagger = morphodita.Tagger.load(self.nlp_model_cs_path)
        self.tokenizer = self.tagger.newTokenizer()


    def tag(self, text):
        self.tokenizer.setText(text)
        tagged_text = TextParse()

        forms = morphodita.Forms()
        lemmas = morphodita.TaggedLemmas()
        tokens = morphodita.TokenRanges()

        while self.tokenizer.nextSentence(forms, tokens):
            self.tagger.tag(forms, lemmas)

            for i in range(len(lemmas)):
                lemma = lemmas[i]
                token = tokens[i]
                token_str = text[token.start: token.start + token.length]
                print(lemma.lemma)
                tag = self.get_tag_from_str(lemma.tag)
                tagged_text.add_tagged_token(token_str, tag)

        print(tagged_text)
        return tagged_text


    def get_tag_from_str(self, tag_str : str) -> Tag:
        tag = Tag()
        tag.pos = POS(tag_str[0])

        return tag



if __name__ == '__main__':
    tagger = Tagger()
    # tagger.tag("Now take the screwdriver and put it in the right corner.")
    tagger.tag("Teď vezmi šroubovák a polož ho do pravého rohu.")