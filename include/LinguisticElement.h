/**	@name LinguisticElement.h
  *
  *
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	0.01.00
  *	@date		2013-03-29
  *
  *	Version History:
  *	@version	0.01.00
  *	@date		2013-03-29
  *	File creation
  *
**/
#ifndef LINGUISTICELEMENTS_H
#define LINGUISTICELEMENTS_H
#include <map>
#include <string>
#include "LinguisticStructures.h"

namespace NLP{
    class LinguisticTree;

    //Type of the linguistic element
    enum type{ root,

        //special word types
        coordConj, determiner, foreign, preposition, listMarker,
        auxiliary, genitiveMark, particle, symbol, to, interjection,
        //noun types
        cardinalNum, noun, pluralNoun, persPronoun, posPronoun,
        //verb types
        verbBase, verbPast, verbGerund, verbPastPart, verbPresent,
        //adjective types
        adjective, adjComp, adjSuper,
        //adverb types
        adverb, advComp, advSuper,

        //phrase types
        sentence, nounPhrase, verbPhrase, prepPhrase, advPhrase, adjPhrase, subClause,
        sinv, phrasalVerbPart,

        numTypes, unrecognizedType
    };

    /**
        Brief Description of the different values of NLP::type

        *Structure Types*
        root: the type of the head node. ROOT maps to root.

        *Word Types*
        *Special*
        coordConj: short for coordinating conjunction. CC maps to coordConj.
            examples: and, both, either, for

        determiner: DT, PDT, and WDT maps to determiner.
            examples: all, an, another, any, both, each, every

        foreign: short for foreign word. FW maps to foreign.

        preposition: subordinate preposition or conjuction. IN maps to preposition
            examples: astride, among, within, towards, for, near, if

        listMarker: LS maps to listMarker.
            examples: A., A, one, first

        auxiliary: short for modal auxiliary. MD maps to auxiliary.
            examples: can, might, need

        genitiveMark: marks defining genetive form. POS maps to genitiveMark.
            examples: ', 's

        particle: RP maps to particle.
            examples: aboard about across

        symbol: SYM, ',', '.', ';', and ':' map to symbol
            examples: %, ^, &

        to: "to" as preposition or infinitive marker
            examples: to

        interjection: UH maps to interjection.
            examples: uh, hey, huh, dammit

        cardinalNum: short for cardinal number. CD maps to cardinalNum.
            examples: nine-thirty, 0.5, '79, 78-degrees

        noun: singular noun (proper or otherwise). NN and NNP map to noun.
            examples: word, America, Michael, boardroom

        pluralNoun: plural noun (proper or otherwise). NNS and NNPS map to pluralNound.
            examples: cars, battles, Templars

        persPronoun: short for personal pronoun. EX, PRP, and WP map to persPronoun.
            examples: I, you, he, she

        posPronoun: short for possessive pronoun. PRP$ and WP$ map to posPronoun.
            examples: his, her, my

        verbBase: base form of a verb. VB maps to verbBase
            examples: ask, assemble

        verbPast: verb past tense. VBD maps to verbPast
            examples: watched, saw

        verbGerund: verb present participle or gerund. VBG maps verbGerund
            examples: watching, waiting

        verbPastPart: verb past participle. VBN maps to verbPastPart
            examples: dilapidated

        verbPresent: verb present tense. VBP, and VBZ maps to verbPresent
            examples: watch, see

        adjective: adjective or ordinal adj. JJ maps to adjective.
            examples: first, ill-mannered

        adjComp: comparitive adjective. JJR maps to adjComp.
            examples: cleaner, better, bolder

        adjSuper: superlative adjective. JJS maps to adjSuper.
            examples: cleanest, best, boldest

        adverb: RB and WRB maps to adverb.
            examples: quickly, occasionally

        advComp: short for comparitive adv. RBR maps to advComp
            examples: faster, stronger

        advSuper: short for superlative adv. RBS maps to advSuper
            examples: fastest, strongest

        *Phrase Types*
        sentence: S maps to sentence

        nounPhrase: NP maps to nounPhrase

        verbPhrase: VP maps to verbPhrase

        prepPhrase: short for preposition phrase. PP maps to prepPhrase

        advPhrase: short for adverb Phrase. ADVP maps to advPhrase

        subClause: short for subordinate clause. SBAR maps to subClause

        numTypes: holds the number of values for NLP::type

        @remarks Please consult the StanfordNLP documentation for more info
            regarding these linguistic types.
    **/

    class Assistant{
        private:
            Assistant();

            static std::map< std::string, NLP::type > typeMap;
            static std::map< std::string, NLP::Dependency::type > depMap;

        public:
            static void init();
            static bool isInit();

            static type getType( const char* partOfSpeech );
            static Dependency::type getDependency( const char* depAbbrev );

            static bool isWord( NLP::type lingType );
    };

    class LinguisticElement{
        public:
            virtual ~LinguisticElement();

        protected:
            LinguisticElement( NLP::type lingType );

            NLP::type lingType;

        public:
            static LinguisticElement* getRoot();
            static LinguisticElement* getUnrecognized();
            static LinguisticElement* getPhrase( NLP::type lingType );

            bool isNoun();
            bool isVerb();
            bool isAdv();
            bool isAdj();

            NLP::type getType();
    };

    class Word:public LinguisticElement{
        public:
            virtual ~Word();

        protected:
            Word( NLP::type lingType, const char* word, unsigned int id );

            char* word;
            unsigned int id;

        public:
            static Word* getWord( NLP::type lingType, const char* word, unsigned int id );

            char* getText(){ return word; }

            unsigned int getID(){ return id; }
    };

}

#endif // LINGUISTICELEMENTS_H
