/**	@name LinguisticElement.cpp
  *
  *
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	1.00.00
  *	@date		2013-03-29
  *
  *	Version History:
  *	@version	0.01.00
  *	@date		2013-03-29
  *	File creation
  *	@version	1.00.00
  *	@date		2013-03-30
  * @author Joe Martinez
  *	Dependency Map has been initialized
  * in NLP::Assistant::init
  *	@version	1.00.01
  *	@date		2013-04-02
  *	Fixed BUFFER OVERFLOW created by Word
  * constructor
  *	@version	1.00.02
  *	@date		2013-04-03
  *	Added support for NP-TMP and QP phrase
  * types in order to interpret cardinals
  *
**/
#include "LinguisticElement.h"
#include <cstdlib>
#include <cstring>
#include <stdexcept>

std::map<std::string, NLP::type> NLP::Assistant::typeMap;
std::map<std::string, NLP::Dependency::type> NLP::Assistant::depMap;

void NLP::Assistant::init(){
    //initialize typeMap;
    typeMap["ROOT"] = NLP::root;
    typeMap["CC"] = NLP::coordConj;
    typeMap["DT"] = NLP::determiner;
    typeMap["PDT"] = NLP::determiner;
    typeMap["WDT"] = NLP::determiner;
    typeMap["FW"] = NLP::foreign;
    typeMap["IN"] = NLP::preposition;
    typeMap["LS"] = NLP::listMarker;
    typeMap["MD"] = NLP::auxiliary;
    typeMap["POS"] = NLP::genitiveMark;
    typeMap["RP"] = NLP::particle;
    typeMap["SYM"] = NLP::symbol;
    typeMap["."] = NLP::symbol;
    typeMap[","] = NLP::symbol;
    typeMap[";"] = NLP::symbol;
    typeMap["TO"] = NLP::to;
    typeMap["UH"] = NLP::interjection;
    typeMap["CD"] = NLP::cardinalNum;
    typeMap["NN"] = NLP::noun;
    typeMap["NNP"] = NLP::noun;
    typeMap["NNS"] = NLP::pluralNoun;
    typeMap["NNPS"] = NLP::pluralNoun;
    typeMap["EX"] = NLP::persPronoun;
    typeMap["PRP"] = NLP::persPronoun;
    typeMap["WP"] = NLP::persPronoun;
    typeMap["PRP$"] = NLP::posPronoun;
    typeMap["WP$"] = NLP::posPronoun;
    typeMap["VB"] = NLP::verbBase;
    typeMap["VBD"] = NLP::verbPast;
    typeMap["VBG"] = NLP::verbGerund;
    typeMap["VBN"] = NLP::verbPastPart;
    typeMap["VBP"] = NLP::verbPresent;
    typeMap["VBZ"] = NLP::verbPresent;
    typeMap["JJ"] = NLP::adjective;
    typeMap["JJR"] = NLP::adjComp;
    typeMap["JJS"] = NLP::adjSuper;
    typeMap["RB"] = NLP::adverb;
    typeMap["WRB"] = NLP::adverb;
    typeMap["RBR"] = NLP::advComp;
    typeMap["RBS"] = NLP::advSuper;
    typeMap["S"] = NLP::sentence;
    typeMap["NP"] = NLP::nounPhrase;
    typeMap["NP-TMP"] = NLP::nounPhrase;
    typeMap["QP"] = NLP::nounPhrase;
    typeMap["VP"] = NLP::verbPhrase;
    typeMap["PP"] = NLP::prepPhrase;
    typeMap["ADVP"] = NLP::advPhrase;
    typeMap["ADJP"] = NLP::adjPhrase;
    typeMap["SBAR"] = NLP::subClause;
    typeMap["SINV"] = NLP::sinv;
    typeMap["PRT"] = NLP::phrasalVerbPart;
    //TODO depMap;
    depMap["abbrev"] = NLP::Dependency::abbrev;
    depMap["acomp"] = NLP::Dependency::acomp;
    depMap["advel"] = NLP::Dependency::advel;
    depMap["advmod"] = NLP::Dependency::advmod;
    depMap["agent"] = NLP::Dependency:: agent;
    depMap["amod"] = NLP::Dependency:: amod;
    depMap["appos"] = NLP::Dependency:: appos;
    depMap["attr"] = NLP::Dependency:: attr;
    depMap["aux"] = NLP::Dependency:: aux;
    depMap["auxpass"] = NLP::Dependency:: auxpass;
    depMap["cc"] = NLP::Dependency:: cc;
    depMap["ccomp"] = NLP::Dependency:: ccomp;
    depMap["complm"] = NLP::Dependency:: complm;
    depMap["conj"] = NLP::Dependency:: conj;
    depMap["cop"] = NLP::Dependency:: cop;
    depMap["csubj"] = NLP::Dependency:: csubj;
    depMap["csubjpass"] = NLP::Dependency:: csubjpass;
    depMap["dep"] = NLP::Dependency:: unrecognizedType;
    depMap["det"] = NLP::Dependency:: det;
    depMap["dobj"] = NLP::Dependency:: dobj;
    depMap["expl"] = NLP::Dependency:: expl;
    depMap["infmod"] = NLP::Dependency:: infmod;
    depMap["iobj"] = NLP::Dependency:: iobj;
    depMap["mark"] = NLP::Dependency:: mark;
    depMap["mwe"] = NLP::Dependency:: mwe;
    depMap["neg"] = NLP::Dependency:: neg;
    depMap["nn"] = NLP::Dependency:: nn;
    depMap["npadvmod"] = NLP::Dependency:: npadvmod;
    depMap["nsubj"] = NLP::Dependency:: nsubj;
    depMap["nsubjpass"] = NLP::Dependency:: nsubjpass;
    depMap["num"] = NLP::Dependency:: num;
    depMap["number"] = NLP::Dependency:: number;
    depMap["parataxis"] = NLP::Dependency:: parataxis;
    depMap["partmod"] = NLP::Dependency:: partmod;
    depMap["pcomp"] = NLP::Dependency:: pcomp;
    depMap["pobj"] = NLP::Dependency:: pobj;
    depMap["poss"] = NLP::Dependency:: poss;
    depMap["possessive"] = NLP::Dependency:: possessive;
    depMap["preconj"] = NLP::Dependency:: preconj;
    depMap["predet"] = NLP::Dependency:: predet;
    depMap["prep"] = NLP::Dependency:: prep;
    depMap["prepc"] = NLP::Dependency:: prepc;
    depMap["prt"] = NLP::Dependency:: prt;
    depMap["punct"] = NLP::Dependency:: punct;
    depMap["purpcl"] = NLP::Dependency:: purpcl;
    depMap["quantmod"] = NLP::Dependency:: quantmod;
    depMap["remod"] = NLP::Dependency:: remod;
    depMap["ref"] = NLP::Dependency:: refer;
    depMap["rel"] = NLP::Dependency:: rel;
    depMap["root"] = NLP::Dependency:: root;
    depMap["tmod"] = NLP::Dependency:: tmod;
    depMap["xcomp"] = NLP::Dependency:: xcomp;
    depMap["xsubj"] = NLP::Dependency:: xsubj;
}

bool NLP::Assistant::isInit(){
    return !typeMap.empty();
}

NLP::type NLP::Assistant::getType( const char* partOfSpeech ){
    try{
        return typeMap.at( partOfSpeech );
    }catch( std::out_of_range oor ){
        return NLP::unrecognizedType;
    }
    return NLP::unrecognizedType;
}

NLP::Dependency::type NLP::Assistant::getDependency( const char* depAbbrev ){
    try{
        return depMap.at( depAbbrev );
    }catch( std::out_of_range oor ){
        return NLP::Dependency::unrecognizedType;
    }
    return NLP::Dependency::unrecognizedType;
}

bool NLP::Assistant::isWord( NLP::type lingType ){
    return ( lingType >= NLP::coordConj && lingType <= NLP::advSuper );
}

NLP::LinguisticElement::LinguisticElement( NLP::type lingType ):lingType(lingType){}

NLP::LinguisticElement::~LinguisticElement(){}

NLP::LinguisticElement* NLP::LinguisticElement::getRoot(){
    return new NLP::LinguisticElement( NLP::root );
}

NLP::LinguisticElement* NLP::LinguisticElement::getUnrecognized(){
    return new NLP::LinguisticElement( NLP::unrecognizedType );
}

NLP::LinguisticElement* NLP::LinguisticElement::getPhrase( NLP::type lingType ){
    if( lingType < NLP::sentence || lingType > NLP::phrasalVerbPart ){
        return NULL;
    }
    return new NLP::LinguisticElement( lingType );
}

bool NLP::LinguisticElement::isNoun(){
    return ( lingType >= NLP::cardinalNum ) && ( lingType <= NLP::posPronoun );
}

bool NLP::LinguisticElement::isVerb(){
    return ( lingType >= NLP::verbBase ) && ( lingType <= NLP::verbPresent );
}

bool NLP::LinguisticElement::isAdj(){
    return ( lingType >= NLP::adjective ) && ( lingType <= NLP::adjSuper );
}

bool NLP::LinguisticElement::isAdv(){
    return ( lingType >= NLP::adverb ) && ( lingType <= NLP::advSuper );
}

NLP::type NLP::LinguisticElement::getType(){
    return lingType;
}

NLP::Word::Word( NLP::type lingType, const char* word, unsigned int id ):NLP::LinguisticElement(lingType),id(id){
    this->word = new char[ strlen( word ) + 1 ];
    strcpy( this->word, word );
}

NLP::Word::~Word(){
    delete[] word;
}

NLP::Word* NLP::Word::getWord( NLP::type lingType, const char* word, unsigned int id  ){
    if( lingType < NLP::coordConj || lingType > NLP::advSuper ){
        return NULL;
    }
    if( word == NULL ){
        return NULL;
    }
    return new NLP::Word( lingType, word, id );
}
