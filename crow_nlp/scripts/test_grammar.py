import os
import copy
import nltk
import numpy as np
from nltk import ngrams
from nltk import ParentedTree
from nltk.stem import WordNetLemmatizer
from termcolor import colored


#maintainer: karla.stepanova@cvut.cz

class LastSent:
    def __init__(self):
        LastSent.OVCs = []
        LastSent.Constraints = []
#class to process sentence which contains constraints
#output is a set of constraints connected with actions and objects
#usecase is demonstration based planning for KUKA robot
class ConstraintNLP:

    def __init__(self, grammar=None):
        self.grammar = None  # type: nltk.CFG
        if grammar is None:
            self.set_grammar()
        else:
            self.set_grammar(grammar)
        self.poses = [] # type: [PoseStamped]
        self.locDict = {}  # type: [str, PoseStamped]
        #    'Loc_name': 'value',
        #    'Location': 'value',
        #    }
        self.idLoc = 1 #initial name id of location #TODO should be saved from previous runs somehow
        self.OVCs = []
        self.HSs = [] #home storages
        self.idOVC = 1 #initial name id of location #TODO should be saved from previous runs somehow
        self.idHS = 1 #initial home storage number #TODO should be saved from previous runs somehow
        self.lastSent = LastSent()
#        self.AGDict = {} # type:[str, GroupedAction]

    def extract_constraints_from_sentence(self, sentence):
        self.sentence = sentence#sentence.split()
        self.tree = self.make_tree_from_sent()
        self.tree = ParentedTree.convert(self.tree)
        #self.time_vec = time_vec

        Constraint = [[0] * 3 for i in range(100)]  # TODO initialization of constraints should be improved
        con_id = -1
        conE_id = -1
        s_val = 0
        e_val = 0
        active_con = []
        active_conE = []
        active_conS = []
        list_to_omit = []
        GO_all = []
        Constraints = []
        i = 1
        j = 1
        GO_list = []
        GO_listE = []
        PickPlaceAction = 0
        GroupActionAction = 0
        stop = 0
        then_active = 0
        #self.poses = poses
        listOVCcur = []
        objects_list = []


        #time_vec_poses = self.extract_time_poses()
        #print ('all_poses_time_vec',self.time_vec_poses)
        #print ('time_vec',self.time_vec)

        #todo should be changed for cases where in one sentence are not only grouped actions
        for objects in self.tree.subtrees(filter=lambda t:t.label() == 'AG'):
            print('adding new grouped action')
            print(objects)

        for objects in self.tree.subtrees(filter=lambda t:t.label() == 'GR'):
            print('adding new general rule')
            print(objects)

        #if sentence is describing storage for objects
        for objects in self.tree.subtrees(filter=lambda t:t.label() == 'HS'):
            print ('home storage')

        #extracting constraints and actions from the bag file
        for objects in self.tree.subtrees(filter=lambda t: t.label() == 'O' or t.label() == 'REL' or t.label() == 'STOP'):
            i = i + 1
            objects_list.append(objects)
            print(objects)

            if objects.label() == 'O':
                O_ID = j
                j = j + 1
                print('sentence ID', O_ID)

            # to create nested relations, we keep a liste of active constraints which are not yet finished (active_con)
            if objects.label() == 'REL':
                    if 'Then' in objects.leaves() or 'then' in objects.leaves():
                        print('Then')

            if objects.label() == 'STOP':
                stop = 1
                print('stop')

        return objects_list

    def make_tree_from_sent(self, sentence = None, grammar = None):
        if sentence is None:
            sentence = self.sentence
        if grammar is None:
            grammar = self.grammar
        # parsing sentence
        rd_parser = nltk.RecursiveDescentParser(grammar)
        for p in rd_parser.parse(sentence):
            tree = p
            return tree
        # return tree

    def set_grammar(self, grammar=None):
        if grammar is None:
            #TODO O PREP O make generative, same for LO
            # self.grammar = nltk.CFG.fromstring("""
            # S -> GO STOP| RELP GGO RELP GGO STOP|RELP GGO RELP GGO RELP GGO STOP |RELP GGO RELP GGO RELP GGO RELP GGO STOP|AG STOP|HS STOP
            # AG -> APP AP|
            # HS -> VP CO CO CO CO LO |VP CO CO CO CO | VP CO |VP CO LO
            # CO -> "corner" L | "corner" NUM L
            # LO -> LLO |LLO PREP LLO|LLO LLO|LLO PREP LLO PREP LLO|LLO LLO LLO|LLO PREP LLO PREP LLO PREP LLO|LLO LLO LLO LLO|LLO PREP LLO PREP LLO PREP LLO PREP LLO|LLO LLO LLO LLO LLO|LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO|LLO LLO LLO LLO LLO LLO|LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO|LLO LLO LLO LLO LLO LLO LLO|LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO|LLO LLO LLO LLO LLO LLO LLO LLO|LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO PREP LLO
            # LLO -> "location" NUM L| "location" L
            # AP -> V DET ADJ N|
            # GO -> O PREP O | O | O PREP O PREP O | O PREP O PREP O PREP O  | O PREP O PREP O PREP O PREP O  | O PREP O PREP O PREP O PREP O PREP O
            # GGO -> TO PREP TO | TO | TO PREP TO PREP TO | TO PREP TO PREP TO PREP TO  | TO PREP TO PREP TO PREP TO PREP TO  | TO PREP TO PREP TO PREP TO PREP TO PREP TO
            # TO -> O|RO
            # O -> V NP | V NP L | L | V NP PREP2 L | V NP PREP2 L PREP2 L |V NP L TP| L TP| V NP PREP2 L TP| V NP PREP2 L PREP2 L TP
            # RO -> RELP O RELP O
            # VP -> V NP | V NP PP
            # NP -> N | DET N| DET ADJ N |DET ADJ N N|ADJ N N
            # RELP -> REL |REL TP
            # REL -> "first" | "then" | "First"
            # STOP -> "stop" | "STOP" | "Stop"
            # N -> "point" | "line" |"bolt"|"it"|"storage"|"action" |"location"|"Bolt"|"Point"
            # APP -> ADJ N V V
            # PREP -> "and"
            # PREP2 -> "from" | "to"
            # V -> "make" | "Make" | "glue" | "Glue" | "Pick" | "pick" | "Place" | "place" |"called"| "is" |"showing"|"Showing"
            # NUM -> "one" | "two" | "three" | "four" | "five" | "six"| "seven" | "eight" | "nine" | "ten" | "eleven" | "twelve"|"1"|"2"|"3"|"4"|"5"|"6"|"7"|"8"|"9"|"10"|"11"|"12"
            # T -> "seconds" | "second" | "minute" | "minutes" | "hour" | "hours" | "day" | "days" | DET T | DET NUM T
            # DET -> "a" | "an" | "the" | "my"|"this"
            # ADJ -> "left" | "right" | "top" | "front" | "upper" | "lower"|"small"|"big"|"Then"|"this"|"This"|"last"|"Last"|"same"|"Same"
            # POS -> "corner" | "border" | "middle" | "here"
            # P -> "in" | "on" | "by" | "at"|"within"
            # L -> P DET POS | P DET ADJ POS |POS |NP
            # TP ->  P NUM T|P DET T
            #  """)
            # self.grammar = nltk.CFG.fromstring("""
            #             SS -> S | AG | GR
            #             S -> OR | OR "CC" OR "CC" OR | OR "CC" OR | OR "CC" OR "CC" OR "CC" OR
            #             OR -> O | REL O | REL O REL O | REL O REL O REL | O REL O | O REL O REL O | REL O O | REL "PRP" O O | "PRP" O |REL O REL O REL O
            #             O -> VP | VP LOCS | TIME VP | TIME VP LOCS | VP HOW | VP BY | VP BY HOW | VP LOCS ADV | VP LOCS ADV TIME | VP TIME | VP LOCS TIME | VP HOW TIME | VP BY TIME | VP BY HOW TIME | VP FOR | VP FOR LOCS | VP FOR TIME | VP FOR LOCS TIME | VP FOR ADV | VP FOR ADV LOCS ADV | VP FOR ADV LOCS ADV TIME | VP BY FOR ADV | VP BY FOR
            #             AG -> "DT" "NN" "VBZ" "VBN" O
            #             GR -> "NNP" "JJ" "NN" S | "NNP" "JJ" "NN"
            #             REL -> "RL"
            #             LOCS -> LOC | "IN" LOC "TO" LOC | "TO" NPC | "IN" NPC | "IN" "PRP" | LOC "CC" LOC | LOC "CC" LOC "CC" LOC | LOC "CC" LOC "CC" LOC "CC" LOC
            #             LOC -> "HERE" | P "DT" NNN | P "DT" "ADJ" NNN | "TO" NP
            #             BY -> "IN" "DT" NNN  | "VBG" "DT" NNN | "IN" "DT" NNN "IN" NNN "CD" | "DT" NNN "IN" NNN "CD" | NNN "IN" NNN "CD"
            #             HOW -> "IN" "DT" "CD" NNN NNN | "IN" "DT" "CD" NNN
            #             FOR -> "TO" VV NPC
            #             NPC -> NP| "PDT" NP
            #             NP -> NNN | "PRP" | "DT" NNN| "DT" ADJ NNN |"DT" ADJ NNN NNN| ADJ NNN NNN | "DT" ADJ| ADJ ADJ NNN | "DT" ADJ ADJ NNN | "DT" "CD" NNN NNN | "DT" "CD" | "DT" NNN NNN | "DT" "CD" NNN | ADJ NNN
            #             NNN -> "NN" | "NNS"
            #             VP -> VV | VV NPC | VV NPC PP | VV ADV | VV NPC ADV | VV NPC PP ADV | VV NPC "CC" NPC | VV NPC "CC" NPC "CC" NPC
            #             VV -> "VB" | "VB" "ON" | "VBG" | "VBP" | "VBG" "VBG" | "VBP" "VBG" | "MD" "VB"
            #             PP -> "P" NPC | "P"
            #             P -> "IN" | "TO" | "ON"
            #             ADJ -> "JJ" | "JJR" | "JJS"
            #             ADV -> "RB" | "RBR" | "RBS"
            #             TIME -> "IN" "CD" NNN
            #              """)
            self.grammar = nltk.CFG.fromstring("""
                                    SS -> S
                                    S -> OR "CC" OR
                                    OR -> O| REL O
                                    O -> VP | VP LOCS | TIME VP | TIME VP LOCS | VP HOW | VP BY | VP BY HOW | VP LOCS ADV | VP LOCS ADV TIME | VP TIME | VP LOCS TIME | VP HOW TIME | VP BY TIME | VP BY HOW TIME | VP FOR | VP FOR LOCS | VP FOR TIME | VP FOR LOCS TIME | VP FOR ADV | VP FOR ADV LOCS ADV | VP FOR ADV LOCS ADV TIME | VP BY FOR ADV | VP BY FOR
                                    AG -> "DT" "NN" "VBZ" "VBN" O
                                    GR -> "NNP" "JJ" "NN" S | "NNP" "JJ" "NN" 
                                    REL -> "RL"
                                    LOCS -> LOC | "IN" LOC "TO" LOC | "TO" NPC | "IN" NPC | "IN" "PRP" | LOC "CC" LOC | LOC "CC" LOC "CC" LOC | LOC "CC" LOC "CC" LOC "CC" LOC
                                    LOC -> "HERE" | P "DT" NNN | P "DT" "ADJ" NNN | "TO" NP
                                    BY -> "IN" "DT" NNN  | "VBG" "DT" NNN | "IN" "DT" NNN "IN" NNN "CD" | "DT" NNN "IN" NNN "CD" | NNN "IN" NNN "CD"
                                    HOW -> "IN" "DT" "CD" NNN NNN | "IN" "DT" "CD" NNN 
                                    FOR -> "TO" VV NPC
                                    NPC -> NP| "PDT" NP 
                                    NP -> NNN | "PRP" | "DT" NNN| "DT" ADJ NNN |"DT" ADJ NNN NNN| ADJ NNN NNN | "DT" ADJ| ADJ ADJ NNN | "DT" ADJ ADJ NNN | "DT" "CD" NNN NNN | "DT" "CD" | "DT" NNN NNN | "DT" "CD" NNN | ADJ NNN
                                    NNN -> "NN" | "NNS"
                                    VP -> VV | VV NPC | VV NPC PP | VV ADV | VV NPC ADV | VV NPC PP ADV | VV NPC "CC" NPC | VV NPC "CC" NPC "CC" NPC
                                    VV -> "VB" | "VB" "ON" | "VBG" | "VBP" | "VBG" "VBG" | "VBP" "VBG" | "MD" "VB"
                                    PP -> "P" NPC | "P"
                                    P -> "IN" | "TO" | "ON"
                                     """)
        else:
            self.grammar = grammar
        return

    def positions_of_word(self, word, sentence=None):
        if sentence is None:
            sentence = self.sentence
        list_words_pos = []
        [list_words_pos.append(i) for i, item in enumerate(sentence) if item == word]
        return list_words_pos

    def position_of_ngrams(self, ngram, sentence = None):
        #gives list of all appearance of ngrams in a sentence
        if sentence is None:
            sentence = self.sentence
        list_ngrams_pos = []
        [list_ngrams_pos.append(i) for i, ng in enumerate(ngrams(sentence, len(ngram))) if ng == tuple(ngram)]
        # list(nltk.ngrams(words, len(ngram))).index(tuple(ngram))
        return list_ngrams_pos

    def find_position_word_exclusive(self, word_array, list_to_omit, sentence = None):
        # finds a first appearance of word in a sentence with omitting already selected positions defined in list_to_omit
        # in parallel it updates list_to_omit
        # INPUT: word_array - i.e. ['in', 'the', 'corner']
        #   sentence - sentence split to words, i.e. "make point here".split()
        #   list_to_omit - list of already selected word positions (locations) to be omitted
        #OUTPUT: list_to_omit - extended list of already selected word positions
        #   word_position - first appearance of word (location idx) in the sentence considering list_to_omit
        if sentence is None:
            sentence = self.sentence
        list_ngrams_pos = self.position_of_ngrams(word_array, sentence)
        list_ngrams_pos_valid = set(list_ngrams_pos).difference(list_to_omit)
        list_to_omit.append(sorted(list(list_ngrams_pos_valid))[0])
        word_position = list_to_omit[-1]
        return list_to_omit, word_position

class NLTK:
    def tag(self, text):
        tag_list = []
        tokens = nltk.word_tokenize(text)
        for pair in nltk.pos_tag(tokens):
            tag_list.append({"word": pair[0], "pos": pair[1]})
        return tag_list

    def lemma(self, text):
        lemmatizer = WordNetLemmatizer()
        lemm_list = []
        text = text.lower()
        for word in text.split():
            lemm_list.append({"word": word, "lemma": lemmatizer.lemmatize(word)})
        return lemm_list


def get_parse_tree(sentence):
    constNLP = ConstraintNLP()

    verbs_list = ['glue', 'insert', 'push', 'move', 'weld', 'drill', 'use', 'press', 'place']
    adj_list = ['blue', 'red', 'green', 'yellow']
    rel_list = ['after', 'first', 'then']
    on_list = ['off', 'on']
    loc_list = ['here']

    sentence_tag = []
    nl_tk = NLTK()
    nltk_output = nl_tk.lemma(sentence)

    nltk_output_tag = nl_tk.tag(sentence)
    for idx, word in enumerate(nltk_output_tag):
        # solving the problem of push, move, insert recognized in the beginning of the sentence as nouns
        if (((nltk_output_tag[idx]['pos'] == 'NNP' or nltk_output_tag[idx]['pos'] == 'NN') or nltk_output_tag[idx][
            'pos'] == 'WRB') or (nltk_output_tag[idx]['pos'] == 'IN')) and (
                nltk_output[idx]['lemma'] in verbs_list):
            if idx > 0:
                if (nltk_output_tag[idx - 1]['pos'] != "DT") and (nltk_output_tag[idx - 1]['pos'] != "JJ"):
                    nltk_output_tag[idx]['pos'] = 'VB'
            else:
                nltk_output_tag[idx]['pos'] = 'VB'
        if nltk_output[idx]['lemma'] in rel_list:
            nltk_output_tag[idx]['pos'] = 'RL'
        if nltk_output[idx]['lemma'] in loc_list:
            nltk_output_tag[idx]['pos'] = 'HERE'
        if nltk_output[idx]['lemma'] in adj_list:
            nltk_output_tag[idx]['pos'] = 'JJ'
        if nltk_output[idx]['lemma'] in on_list:
            nltk_output_tag[idx]['pos'] = 'ON'

        sentence_tag.append(nltk_output_tag[idx]['pos'])

    constNLP.extract_constraints_from_sentence(sentence_tag)
    return constNLP.tree

if __name__ == '__main__':

    constNLP = ConstraintNLP()

    #nltk.download('wordnet')
    #nltk.download('punkt')
    #nltk.download('averaged_perceptron_tagger')

    sentences = []
    sentences.append('First make a point here and then make a point here')
    sentences.append('make a point here and make a point in the corner')
    sentences.append('Make a line from here to here')
    sentences.append('First make a line from here to here and then glue a point here')
    sentences.append('First make a line from here to here and then within 2 seconds glue a point here')
    sentences.append('First make a line from here to here then within 2 seconds glue a point here')
    sentences.append('Take a screwdriver and give it to me')
    sentences.append('Push it slowly towards me')
    sentences.append('Push the button')
    sentences.append('Pushing this box slightly forward')
    sentences.append('Gluing this paper on the wall')
    sentences.append('Inserting the piece in the middle')
    sentences.append('Push this box slightly forward')
    sentences.append('Glue this paper on the wall')
    sentences.append('Insert the piece in the middle')
    sentences.append('Insert the piece to the corner')
    sentences.append('Insert the middle piece')
    sentences.append('Insert the piece to the hole')
    sentences.append('Insert the piece by the red tool')
    sentences.append('Insert the turimumb in the middle')
    sentences.append('Weld a joint by the blowtorch under a 90 degree angle')
    sentences.append('Weld a joint using the blowtorch under a 90 degree angle')
    sentences.append('Weld a joint using the blowtorch')
    sentences.append('Weld a joint under a 90 degree angle')
    sentences.append('Move all screwdrivers to the storage')
    sentences.append('Teaching new general rule')
    sentences.append('Glue a point here and here')
    sentences.append('Glue a point here and here and here')
    sentences.append('Glue a point here and here then glue a point here')
    sentences.append('Glue a point here and here Then pick a small cube')
    sentences.append('Glue a point here and here Then pick a small cube and then place it to the same place')
    sentences.append('Glue a point here and here Then pick a small cube and put it to the same place')
    sentences.append('This action is called glue a cube')
    sentences.append('New general rule')
    sentences.append('New general rule move machine to the corner')
    sentences.append('New general rule move machine to the corner after finishing')
    sentences.append('New general rule after finishing work move the machine to the corner')
    sentences.append('Move the hammer and nails')
    sentences.append('Move the hammer and all the nails')
    sentences.append('Move the hammer and all the nails to the right and pick the hammer')
    sentences.append('Drill a five centimeter hole')
    sentences.append('Drill a five centimeter hole with a drill of size 4')
    sentences.append('Move all the things in the workspace')
    sentences.append('Glue these two together')
    sentences.append('Move all the things in the workspace away')
    sentences.append('Glue these two together for five seconds')
    sentences.append('Use a glue of type 2')
    sentences.append('Use a glue of type 2 to glue all the pieces together')
    sentences.append('Insert the piece to the left corner')
    sentences.append('Use the blue drill')
    sentences.append('Use the blue drill to drill a hole here')
    sentences.append('Use the blue drill to drill a hole here for 10 seconds')
    sentences.append('Use blue glue to glue these two things')
    sentences.append('switch off the welding machine')
    sentences.append('After finishing welding switch off the welding machine')
    sentences.append('Press a power button')
    sentences.append('After you finish welding switch off the welding machine')
    sentences.append('You should insert all the nails in the box')
    sentences.append('After you finish welding switch off the welding machine and press a power button')
    sentences.append('First make point here then make point here then make point here')
    sentences.append('You should move the black hammer and the green hammer into the box')
    sentences.append('place it to the same place')
    sentences.append('Glue a  point here and here then pick a small cube and place it to the same location')
    sentences.append('Glue a point here and here Then pick a small cube and place it to the same place')
    sentences.append('You should move the black hammer and the green hammer into the box')
    sentences.append('You should move the black hammer and the green hammer into the box now')
    #sentences.append('You should now move the black hammer and the green hammer into the box')
    # sentences.append('First make point here and make point here then make point here then make point in the corner')
    # sentences.append('First make point here then make point in the corner stop and make a point here or make a point here')
    #sentences.append('First make point here and make point here and first make point in the corner then make point here then make point here and make point here and first make point here then make point here')
    # sentences.append('Make point here and make point here and make point here and make point here and make point here and make point here')

    print_lemma = 1
    verbs_list = ['glue','insert','push','move', 'weld', 'drill','use','press','place']
    adj_list = ['blue','red','green','yellow']
    rel_list = ['after','first','then']
    on_list = ['off','on']
    loc_list = ['here']

    for sentence in sentences:
    #sentence = sentences[0]
        sentence_tag = []
        nl_tk = NLTK()
        nltk_output = nl_tk.lemma(sentence)

        nltk_output_tag = nl_tk.tag(sentence)
        if print_lemma == 1:
            #print(nltk_output)
            print("{:<15} {:<15} {:<15}".format('Word','NLTK_lemma', 'NLTK_tag'), "\n", '-' * 60)
        for idx, word in enumerate(nltk_output_tag):
            #solving the problem of push, move, insert recognized in the beginning of the sentence as nouns
            if (((nltk_output_tag[idx]['pos'] == 'NNP' or nltk_output_tag[idx]['pos'] == 'NN')or nltk_output_tag[idx]['pos'] == 'WRB') or (nltk_output_tag[idx]['pos'] == 'IN')) and (nltk_output[idx]['lemma'] in verbs_list):
                if idx>0:
                    if (nltk_output_tag[idx-1]['pos'] != "DT") and (nltk_output_tag[idx-1]['pos'] != "JJ"):
                        nltk_output_tag[idx]['pos'] = 'VB'
                else:
                    nltk_output_tag[idx]['pos'] = 'VB'
            if nltk_output[idx]['lemma'] in rel_list:
                nltk_output_tag[idx]['pos'] = 'RL'
            if nltk_output[idx]['lemma'] in loc_list:
                nltk_output_tag[idx]['pos'] = 'HERE'
            if nltk_output[idx]['lemma'] in adj_list:
                nltk_output_tag[idx]['pos'] = 'JJ'
            if nltk_output[idx]['lemma'] in on_list:
                nltk_output_tag[idx]['pos'] = 'ON'

            if print_lemma:
                print("{:<15}".format(word["word"]),
                        "{:<15} {:<15}".format(nltk_output[idx]['lemma'], nltk_output_tag[idx]['pos']))
            sentence_tag.append(nltk_output_tag[idx]['pos'])

        #print("Number of words: ", num_words)

        print(sentence_tag)

        objects = constNLP.extract_constraints_from_sentence(sentence_tag)
        print('sentence: {}'.format(sentence))
        for ob in objects:
            print(ob)