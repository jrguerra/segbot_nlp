/**	@name LinguisticTree.cpp
  *
  *	LinguisticTree is part of the NLP package
  * as a data structure holding the information
  * gained from stanford NLP
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	1.00.03
  *	@date		2013-04-02
  *
  *	Version History:
  *	@version	1.00.01
  *	@date		2013-03-29
  *	Fixed possibility of linguisticType ending in "("
  * rather than a space.
  *	@version	1.00.02
  *	@date		2013-03-30
  *	Added some extra safety features to protect NLP::LT::
  * interpreter from segfaulting inside getType().
  *	@version	1.00.03
  *	@date		2013-04-02
  *	Added possibility of there being no space between
  * LinguisticType and the word of that type. Also made
  * the string that holds linguisticType be of dynamic size
  * so no more BUFFER OVERFLOW if something else goes wrong.
  * Hopefully, the last fix
  *	@version	1.00.04
  *	@date		2013-04-12
  *	Added support for ROOT type dependencies.
  *	@version	1.00.05
  *	@date		2013-04-18
  *	Added getLastChild
**/
#include "LinguisticTree.h"
#include "LinguisticElement.h"
#include "LinguisticStructures.h"
#include "pugixml.hpp"
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <ctype.h>

using namespace std;

NLP::LinguisticTree::LinguisticTree():head(NULL){
}

NLP::LinguisticTree::LinguisticTree( const char* filename ){

    head = new NLP::LinguisticTree::PhraseNode( NULL, NLP::LinguisticElement::getRoot() );

    //make sure the typeMap is initiated
    if( !NLP::Assistant::isInit() ){
        NLP::Assistant::init();
    }

    //parse the document
    pugi::xml_document doc;
    pugi::xml_parse_result parseResult = doc.load_file( filename );
    if( parseResult.status != pugi::status_ok ){
        return;
    }
    //grab the "sentences" node, harcoded result is fastest
    pugi::xml_node sentencesNode = doc.first_child().first_child().first_child();
    //if the document has no sentences
    if( sentencesNode.empty() ) return;

    //grab first sentence
    int currentSentenceID = 0;
    pugi::xml_node searcher = sentencesNode.first_child();
    //loop handling sentences
    while( !strcmp( searcher.name(), "sentence" ) ){
        //values for token range
        int first = -1, last;

        //grab first token
        pugi::xml_node tokens = searcher.first_child();
        pugi::xml_node token = tokens.first_child();
        if( !token.empty() ){
            first = words.size();
        }
        //loop handling tokens
        while( !strcmp( token.name() , "token" ) ){
            //grab the part of speech
            const char* partOfSpeech = token.child("POS").first_child().value();
            //Get it's enumeration
            NLP::type wordType = NLP::Assistant::getType( partOfSpeech );
            if( wordType == NLP::unrecognizedType ){
                //upon error
                cerr<<"Error: unrecognized type: "<<partOfSpeech<<endl;
            }
            //get the word data
            const char* wordText = token.first_child().first_child().value();
            //store the new word
            words.push_back( new WordNode( NULL, Word::getWord( wordType, wordText, words.size() ) ) );

            //get the next token
            token = token.next_sibling();
        }
        last = words.size();
        //Save the range of the sentence
        sentences.push_back( NLP::Range( first, last ) );

        //Handle the parsing:
        //grab the parse node;
        pugi::xml_node parsing = tokens.next_sibling();
        if( parsing.empty() ) return;
        parseData( parsing.first_child().value(), currentSentenceID );

        //Handle Dependencies
        //grab the basic dependencies node
        pugi::xml_node depNode = parsing.next_sibling();

        pugi::xml_node depend = depNode.first_child();
        while( !depend.empty() ){
            //grab the dependencies values
            NLP::Dependency::type identification = NLP::Assistant::getDependency( depend.first_attribute().value() );
            if( identification == NLP::Dependency::unrecognizedType ){
                cerr<<"Error: Unrecognized Dependency Type: "<<depend.first_attribute().value()<<endl;
            }

            pugi::xml_node governor = depend.first_child();
            pugi::xml_node dependent = governor.next_sibling();

            if( governor.first_attribute().as_int(0) != 0 ){
                int governorID = governor.first_attribute().as_int(0) + (sentences[ currentSentenceID ]).getFirst() - 1;
                int dependentID = dependent.first_attribute().as_int(0) + (sentences[ currentSentenceID ]).getFirst() - 1;


                if( governorID != -1 ){
                    //store the dependencies in their elements
                    words[governorID]->addGoverned( dependencies.size() );
                    words[dependentID]->addDependent( dependencies.size() );

                    //store the dependency in the tree
                    dependencies.push_back( Dependency( identification, governorID, dependentID ) );
                }
            }
            //grab the next dependency
            depend = depend.next_sibling();
        }
        searcher = searcher.next_sibling();
        currentSentenceID++;
    }
    pugi::xml_node coreferenceNode = sentencesNode.next_sibling();
    if( !strcmp( coreferenceNode.name(), "coreference" ) ){
        pugi::xml_node reference = coreferenceNode.first_child();
        int referenceID = 0;
        while( !reference.empty() ){
            coreferences.push_back( vector<NLP::Range>() );
            pugi::xml_node mention = reference.first_child();
            while( !mention.empty() ){
                int mentionID = 0;
                //grab sentence number
                pugi::xml_node finder = mention.first_child();
                int sentenceID = atoi(finder.first_child().value()) - 1;
                //grab first range value.
                finder = finder.next_sibling();
                int rangeBegin = atoi( finder.first_child().value() ) - 1;
                //grab end range value.
                finder = finder.next_sibling();
                int rangeEnd = atoi( finder.first_child().value() ) - 1;

                //push the mention on the current coreference.
                coreferences[referenceID].push_back(
                    NLP:: Range(sentences[sentenceID].getFirst() + rangeBegin, sentences[sentenceID].getFirst() + rangeEnd ) );
                //add coreferences to the head member
                finder = finder.next_sibling();
                int headMember = atoi( finder.first_child().value() ) - 1;

                NLP::CoreferenceIdentifier identifier( referenceID, mentionID );
                words[ sentences[sentenceID].getFirst() + headMember ]->addCoreference( identifier );

                mentionID++;
                mention = mention.next_sibling();
            }

            reference = reference.next_sibling();
            referenceID++;
        }
    }
}

NLP::LinguisticTree::~LinguisticTree(){
    if( head != NULL ){
        delete head;
    }
}

void NLP::LinguisticTree::parseData( const char* data, int sentenceID ){
    //get a copy of the data for destructive purposes;
    char* dataCopy = new char[ strlen(data) + 1 ];
    strcpy( dataCopy, data );

    //set's an iterator to the first word in the current sentence
    vector<WordNode*>::iterator wordIterator = words.begin();
    wordIterator[ sentences[sentenceID].getFirst() ];
    int paren = strcspn( dataCopy, "()" );

    char* traveller = dataCopy + paren + 1;

    int space = strcspn( traveller, " " );

    traveller += space;

    depthTravel( head, traveller, wordIterator );

    delete[] dataCopy;
}

char* NLP::LinguisticTree::depthTravel( NLP::LinguisticTree::Node* element, char* data, vector<WordNode*>::iterator& i ){
    while( 1 ){
        //find a parenthesis
        int paren = strcspn( data, "()" );
        //if it's not entering a new node, then we are exiting the current one
        if( data[paren] == ')' ){
           return data + paren + 1;
        }else if ( ! data[paren] ){
            //NULL handler just in case
            return data + paren;
        }
        //otherwise find it's linguistic classification
        data += paren + 1;

        int spaceOrParen = strcspn( data, " (" );
        char* linguisticType = new char[ spaceOrParen + 1 ];
        strncpy( linguisticType, data, spaceOrParen );
        linguisticType[ spaceOrParen ] = 0;
        //handles for the possibility of a missing space
        for( int j = 0 ; j < spaceOrParen; ++j ){
            if( islower( linguisticType[j] ) ){
                linguisticType[j] = 0;
                break;
            }
        }

        //make sure the type exists
        NLP::type lingType = NLP::Assistant::getType( linguisticType );
        if( lingType == NLP::unrecognizedType ){
            cout<<"Error: unrecogonized type: "<<linguisticType<<endl;
        }
        delete linguisticType;

        NLP::LinguisticTree::Node* child;
        //if it is a word type it's the next word in the list
        if( NLP::Assistant::isWord( lingType ) ){
            child = *(i++);
            child->setParent( element );
        }else{
            //otherwise it is a new phrase node with children.
            child = new NLP::LinguisticTree::PhraseNode( element, NLP::LinguisticElement::getPhrase( lingType ) );
        }
        data = depthTravel( child, data, i );
    }
}

NLP::LinguisticTree::Node::Node():parent(NULL),parentIndex(0),element(NULL){}

NLP::LinguisticTree::Node::Node( NLP::LinguisticTree::Node* parent, NLP::LinguisticElement* element ):
    parent(parent),element(element){

    if( parent != NULL ){
        parentIndex = parent->addChild( this );
    }
}

NLP::LinguisticTree::Node::~Node(){
    if( element != NULL ){
        delete element;
    }
}

NLP::LinguisticElement* NLP::LinguisticTree::Node::getElement(){ return element; }

NLP::type NLP::LinguisticTree::Node::getType(){
    if( element == NULL ){
        return NLP::unrecognizedType;
    }
    return element->getType();
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::Node::forceNext(){
    //recursive next function
    //if no parent then we're done
    if( parent == NULL ) return NULL;
    //get next sibling
    NLP::LinguisticTree::Node* sibling = parent->getSelectChild( parentIndex+1 );
    if( sibling != NULL ){
        return sibling;
    }
    //if the sibling doesn't exist return parent's next sibling
    return parent->forceNext();
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::Node::getPrev(){
    //if the node doesn't have a parent
    if(parent == NULL) return NULL;
    //if it is the first child then the parent is the previous node
    if( parentIndex == 0 ) return parent;
    //otherwise return the rightmost decendent of the previous sibling
    return parent->getSelectChild( parentIndex - 1 )->getRightMostDecendent();
}

bool NLP::LinguisticTree::Node::setParent( NLP::LinguisticTree::Node* parent ){
    if( this->parent != NULL ){
        return 0;
    }
    this->parent = parent;
    if( parent != NULL ){
        parentIndex = parent->addChild( this );
        if( parentIndex != LINGUISTIC_CHILD_ADD_FAILURE ){
            return 1;
        }
    }
    return 0;
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::Node::getNextSibling(){
    return parent->getSelectChild( parentIndex + 1 );
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::Node::getPrevSibling(){
    return parent->getSelectChild( parentIndex - 1 );
}

NLP::LinguisticTree::PhraseNode::PhraseNode(){}

NLP::LinguisticTree::PhraseNode::PhraseNode( NLP::LinguisticTree::Node* parent, NLP::LinguisticElement* element ):
    Node( parent, element ){}

NLP::LinguisticTree::PhraseNode::~PhraseNode(){
    while( !children.empty() ){
        Node* top = children.back();
        children.pop_back();
        //delete children
        delete top;
    }
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::PhraseNode::getNext(){
    //if a child exists
    if( children.size() ){
        //return the first child
        return children[0];
    }
    //otherwise force the next sibling
    return forceNext();
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::PhraseNode::getSelectChild( int index ){
    //if the requested index is too small or too large retun NULL
    if( ((unsigned int) index) >= children.size() ) return NULL;
    //otherwise return the select child
    return children[index];
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::PhraseNode::getRightMostDecendent(){
    //if no children
    if( children.size() == 0 ) return this;
    //otherwise return the rightmost descendent of the rightmost child
    NLP::LinguisticTree::Node* lastChild = children.back();
    return lastChild->getRightMostDecendent();
}

unsigned int NLP::LinguisticTree::PhraseNode::getNumChildren(){
    return children.size();
}

int NLP::LinguisticTree::PhraseNode::addChild( NLP::LinguisticTree::Node* child ){
    children.push_back(child);
    return children.size()-1;
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::PhraseNode::getFirstChild(){
    return children[0];
}

NLP::LinguisticTree::WordNode::WordNode(){}

NLP::LinguisticTree::WordNode::WordNode( NLP::LinguisticTree::Node* parent, Word* word ):Node(parent, word){}

NLP::LinguisticTree::WordNode::~WordNode(){}

NLP::LinguisticTree::Node* NLP::LinguisticTree::WordNode::getNext(){
    return forceNext();
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::WordNode::getSelectChild( int index ){
    return NULL;
}

unsigned int NLP::LinguisticTree::WordNode::getNumChildren(){
    return 0;
}

NLP::LinguisticTree::Node* NLP::LinguisticTree::WordNode::getRightMostDecendent(){
    //since the wordNode hasno children, it is it's rightmost decendent
    return this;
};

int NLP::LinguisticTree::WordNode::addChild( NLP::LinguisticTree::Node* child ){
    return LINGUISTIC_CHILD_ADD_FAILURE;
}

void NLP::LinguisticTree::WordNode::addGoverned( int i ){
    governedDeps.push_back( i );
}

void NLP::LinguisticTree::WordNode::addDependent( int i ){
    dependentDeps.push_back( i );
}

void NLP::LinguisticTree::WordNode::addCoreference( const NLP::CoreferenceIdentifier& coreference ){
    coreferences.push_back( coreference );
}

NLP::LinguisticTree::Iterator::Iterator():node(NULL),tree(NULL){}

NLP::LinguisticTree::Iterator::Iterator( const Iterator& reference ):node( reference.node ),tree(reference.tree){}

NLP::LinguisticTree::Iterator::~Iterator(){}

NLP::LinguisticTree::Iterator::Iterator(NLP::LinguisticTree::Node* node, NLP::LinguisticTree* tree ):node(node), tree(tree){}

NLP::LinguisticTree::Iterator& NLP::LinguisticTree::Iterator::operator=( const NLP::LinguisticTree::Iterator& reference ){
    node = reference.node;
    tree = reference.tree;
    return *this;
}

NLP::LinguisticTree::Iterator::Iterator( NLP::LinguisticTree* tree ):node(NULL),tree(tree){}

NLP::LinguisticElement* NLP::LinguisticTree::Iterator::operator*(){
    if( node == NULL ) return NULL;
    return node->getElement();
}

NLP::LinguisticTree::Iterator& NLP::LinguisticTree::Iterator::operator++(){
    if( node != NULL ){
        node = node->getNext();
    }
    return *this;
}

NLP::LinguisticTree::Iterator& NLP::LinguisticTree::Iterator::operator--(){
    if( node != NULL ){
        node = node->getPrev();
    }
    return *this;
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::operator++(int){
    NLP::LinguisticTree::Iterator result(node, tree);
    if( node != NULL ){
        node = node->getNext();
    }
    return result;
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::operator--(int){
    NLP::LinguisticTree::Iterator result(node, tree);
    if( node != NULL ){
        node = node->getPrev();
    }
    return result;
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::begin(){
    return LinguisticTree::Iterator( head, this );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::end(){
    return LinguisticTree::Iterator( NULL, this );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::word( unsigned int index ){
    if( index >= words.size() ) return end();
    return NLP::LinguisticTree::Iterator( words[index], this );
}

const char* NLP::LinguisticTree::getText( NLP::LinguisticTree::Iterator& wordIter ){
    if( *wordIter == NULL ) return "";
    if( !NLP::Assistant::isWord( (*wordIter)->getType() ) ) return "";
    return ((NLP::Word*) *wordIter)->getText();
}

unsigned int NLP::LinguisticTree::Iterator::getNumChildren(){
    if( node == NULL ) return -1;
    return node->getNumChildren();
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getNextSibling(){
    if( node == NULL ) return NLP::LinguisticTree::Iterator( tree );
    return NLP::LinguisticTree::Iterator( node->getNextSibling(), tree );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getPrevSibling(){
    if( node == NULL ) return NLP::LinguisticTree::Iterator( tree );
    return NLP::LinguisticTree::Iterator( node->getPrevSibling(), tree );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getFirstChild(){
    if( node == NULL ) return NLP::LinguisticTree::Iterator( tree );
    return NLP::LinguisticTree::Iterator( node->getFirstChild(), tree );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getChild( int index ){
    if( node == NULL ) return NLP::LinguisticTree::Iterator( tree );
    return NLP::LinguisticTree::Iterator( node->getSelectChild( index ), tree );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getLastChild(){
    return getChild( getNumChildren() - 1 );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getParent(){
    if( node == NULL ) return NLP::LinguisticTree::Iterator( tree );
    return NLP::LinguisticTree::Iterator( node->getParent(), tree );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getNextWord(){
    if( tree == NULL ) return getEnd();
    //if the iterator isn't on a word, return failure
    if( !NLP::Assistant::isWord( node->getType() ) ) return tree->end();
    //get the index of the next word
    unsigned int index = ( (NLP::Word*) (node->getElement()) )->getID() + 1;
    return tree->word( index );
}

NLP::LinguisticTree::Iterator NLP::LinguisticTree::Iterator::getPrevWord(){
    if( tree == NULL ) return getEnd();
    //if the iterator isn't on a wor return failure
    if( !NLP::Assistant::isWord( node->getType() ) ) return tree->end();
    unsigned int index = ( (NLP::Word*) (node->getElement()) )->getID() - 1 ;
    return tree->word( index );
}
