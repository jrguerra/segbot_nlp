/**	@name LinguisticStructures.cpp
  *
  *	Contains range and Dependency functions
  * for the LT class
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	0.01.01
  *	@date		2013-03-29
  *
  *	Version History:
  *	@version	0.01.01
  *	@date		2013-03-29
  *	File creation
  *
**/
#include "LinguisticStructures.h"

NLP::Range::Range():startIndex(LINGUISTIC_RANGE_INVALID){}

NLP::Range::Range( int value ):startIndex(value),endIndex(value+1){
    if( startIndex < 0 ){
        startIndex = LINGUISTIC_RANGE_INVALID;
    }
}

NLP::Range::Range( int startIndex, int endIndex ){
    if( startIndex < 0 || endIndex <= startIndex ){
        this->startIndex = LINGUISTIC_RANGE_INVALID;
    }else{
        this->startIndex = startIndex;
        this->endIndex = endIndex;
    }
}

int NLP::Range::getFirst(){
    return startIndex;
}

bool NLP::Range::contains( int index ){
    if( !isValid() ) return 0;

    return ( index >= startIndex ) && ( index < endIndex );
}

bool NLP::Range::setRange( int begin, int end ){
    if( begin < 0 || end <= begin ){
        return false;
    }
    startIndex = begin;
    endIndex = end;
    return true;
}

bool NLP::Range::isValid(){
    return startIndex != LINGUISTIC_RANGE_INVALID;
}

NLP::Dependency::Dependency( NLP::Dependency::type identification, int governor, int dependent ):
    identification(identification),governor(governor),dependent(dependent){
}

NLP::Dependency::type NLP::Dependency::getType(){
    return identification;
}

int NLP::Dependency::getGovernor(){
    return governor;
}

int NLP::Dependency::getDependent(){
    return dependent;
}

NLP::CoreferenceIdentifier::CoreferenceIdentifier( int coref, int index ):coref(coref),corefIndex(index){}
