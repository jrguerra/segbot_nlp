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
                    virtual LinguisticTree::Node* getNext()=0;
                    virtual LinguisticTree::Node* getPrev();
                    virtual LinguisticElement* getElement();
                    virtual LinguisticTree::Node* getSelectChild( int index )=0;
                    virtual int addChild( LinguisticTree::Node* child )=0;
                    bool setParent( LinguisticTree::Node* parent );
                    LinguisticTree::Node* getParent();
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

                    void addGoverned( int i );
                    void addDependent( int i );
                    void addCoreference( const NLP::CoreferenceIdentifier& coreference );
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
                    LinguisticTree::Node* node;

                public:
                    friend bool operator ==( const Iterator& reference1, const Iterator& reference2 ){
                        return reference1.node == reference2.node;
                    }
                    friend bool operator !=( const Iterator& reference1, const Iterator& reference2 ){
                        return reference1.node != reference2.node;
                    }
                    Iterator& operator=( const Iterator& reference );

                    NLP::LinguisticElement* operator*();
                    //TODO overload ->
                    Iterator& operator ++();
                    Iterator& operator --();


            };
            friend class LinguisticTree::Iterator;


    };
}

#endif // LINGUISTICTREE_H
