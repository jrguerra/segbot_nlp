/**	@name LinguisticTree.h
  *
  *	LinguisticTree Header
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	1.00.01
  *	@date		2013-03-29
  *
  *	Version History:
  *	@version	1.00.00
  *	@date		2013-03-29
  *	File creation
  * Currently Beta
  *	@version	1.00.01
  *	@date		2013-03-30
  *	added NLP::LT::Node::getType()
  *
**/
#ifndef LINGUISTICTREE_H
#define LINGUISTICTREE_H
#include <vector>
#include "LinguisticElement.h"
#include "LinguisticStructures.h"

#define LINGUISTIC_CHILD_ADD_FAILURE -1

namespace NLP{
    class LinguisticTree{
        public:
            //ctors
            LinguisticTree();
            LinguisticTree( const char* filename );
            ~LinguisticTree();

        private:
            //internals and private classes
            class Node{
                public:
                    Node();
                    Node( LinguisticTree::Node* parent, LinguisticElement* element );
                    virtual ~Node();

                protected:
                    LinguisticTree::Node* parent;
                    int parentIndex;
                    LinguisticElement* element;

                public:
                    //Gets the next linguistic element in the tree (preorder)
                    virtual LinguisticTree::Node* getNext()=0;
                    //Get the previous elemenent
                    virtual LinguisticTree::Node* getPrev();
                    LinguisticElement* getElement();
                    NLP::type getType();
                    virtual LinguisticTree::Node* getSelectChild( int index )=0;
                    virtual int addChild( LinguisticTree::Node* child )=0;
                    virtual unsigned int getNumChildren()=0;
                    bool setParent( LinguisticTree::Node* parent );
                    LinguisticTree::Node* getParent(){ return parent; }

                    LinguisticTree::Node* getNextSibling();
                    LinguisticTree::Node* getPrevSibling();
                    virtual LinguisticTree::Node* getFirstChild(){ return NULL; }

                    int getParentIndex(){ return parentIndex; }

                protected:
                    LinguisticTree::Node* forceNext();
                public:
                    virtual LinguisticTree::Node* getRightMostDecendent() = 0;
            };
            class PhraseNode:public LinguisticTree::Node{
                public:
                    PhraseNode();
                    PhraseNode( LinguisticTree::Node* parent, LinguisticElement* element );
                    ~PhraseNode();

                private:
                    std::vector<LinguisticTree::Node*> children;

                public:
                    virtual LinguisticTree::Node* getNext();
                    virtual LinguisticTree::Node* getSelectChild( int index );
                    virtual int addChild( LinguisticTree::Node* child );
                    virtual unsigned int getNumChildren();

                    virtual LinguisticTree::Node* getFirstChild();

                protected:
                    virtual LinguisticTree::Node* getRightMostDecendent();
            };
            class WordNode:public LinguisticTree::Node{
                public:
                    WordNode();
                    WordNode( LinguisticTree::Node* parent, Word* word );
                    ~WordNode();

                private:
                    std::vector< int > governedDeps;
                    std::vector< int > dependentDeps;
                    std::vector< NLP::CoreferenceIdentifier > coreferences;

                public:
                    virtual LinguisticTree::Node* getNext();
                    virtual LinguisticTree::Node* getSelectChild( int index );
                    virtual int addChild( LinguisticTree::Node* child );
                    const char* getWord(){ return ((Word*) element)->getText(); }
                    virtual unsigned int getNumChildren();

                    void addGoverned( int i );
                    void addDependent( int i );
                    void addCoreference( const NLP::CoreferenceIdentifier& coreference );

                protected:
                    virtual LinguisticTree::Node* getRightMostDecendent();
            };

            Node* head;
            std::vector< NLP::LinguisticTree::WordNode* > words;
            std::vector< NLP::Range > sentences;

            std::vector<NLP::Dependency> dependencies;
            std::vector< std::vector<NLP::Range> > coreferences;

            void parseData( const char* data, int sentenceID );
            char* depthTravel( NLP::LinguisticTree::Node* element, char* data, std::vector<WordNode*>::iterator& i );

        public:
            class Iterator:public std::bidirectional_iterator_tag{
                public:
                    Iterator();
                    Iterator( const Iterator& reference );
                    ~Iterator();

                private:
                    Iterator( LinguisticTree::Node* n, LinguisticTree* tree );
                    Iterator( LinguisticTree* tree );
                    LinguisticTree::Node* node;
                    LinguisticTree* tree;

                    friend class LinguisticTree;

                public:
                    friend bool operator ==( const Iterator& reference1, const Iterator& reference2 ){
                        return reference1.node == reference2.node && reference1.tree == reference2.tree;
                    }
                    friend bool operator !=( const Iterator& reference1, const Iterator& reference2 ){
                        return reference1.node != reference2.node || reference1.tree != reference2.tree;
                    }
                    Iterator& operator=( const Iterator& reference );

                    NLP::LinguisticElement* operator*();
                    //TODO overload ->
                    Iterator& operator ++();
                    Iterator& operator --();
                    Iterator operator++(int);
                    Iterator operator--(int);

                    unsigned int getNumChildren();
                    Iterator getNextSibling();
                    Iterator getPrevSibling();
                    Iterator getFirstChild();
                    Iterator getChild(int index);
                    Iterator getParent();
                    int getParentIndex(){
                        return node->getParentIndex();
                    }
                    Iterator getNextWord();
                    Iterator getPrevWord();
                    Iterator getEnd(){ return tree->end(); }
            };
            friend class LinguisticTree::Iterator;

            LinguisticTree::Iterator begin();
            LinguisticTree::Iterator end();

            static const char* getText( NLP::LinguisticTree::Iterator& wordIter );

            LinguisticTree::Iterator word( unsigned int index );

    };
}

#endif // LINGUISTICTREE_H
