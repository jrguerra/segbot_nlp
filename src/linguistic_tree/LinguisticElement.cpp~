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
    typeMap["VP"] = NLP::verbPhrase;
    typeMap["PP"] = NLP::prepPhrase;
    typeMap["ADVP"] = NLP::advPhrase;
    typeMap["SBAR"] = NLP::subClause;
    //TODO depMap;
    depMap["abbrev"] = NLP::Dependency::abbrev;
    depMap["advmod"] = NLP::Dependency::advmod;
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

NLP::LinguisticElement* NLP::LinguisticElement::getPhrase( NLP::type lingType ){
    if( lingType < NLP::sentence || lingType > NLP::subClause ){
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
    this->word = new char[strlen( word )];
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
