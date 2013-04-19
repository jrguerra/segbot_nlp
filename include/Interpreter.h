/**	@name Interpreter.h
  *
  *
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	0.02.01
  *	@date		2013-04-03
  *
  *	Version History:
  *	@version	0.01.01
  *	@date		2013-03-29
  *	File creation
  *	@version	0.01.02
  *	@date		2013-03-30
  *	Added debugging tools for CommandVector
  *	@version	0.02.01
  *	@date		2013-04-03
  *	Added internal classes for conversion factors
  * inside interpretter. Also added conversion map
  * and Cardinal Number function.
  *	@version	1.00.00
  *	@date		2013-04-15
  *	Added Gerund Functions
  *	@version	1.00.01
  *	@date		2013-04-18
  *	added functionality for more prepositions.
**/
#ifndef INTERPRETER_H
#define INTERPRETER_H

#include "LinguisticTree.h"
#include <list>
#include <map>
#include <set>
#include <string>
#include <iostream>

enum fullCommand{ FC_move, FC_turn, FC_goLocation, FC_goBack, FC_dance, FC_numFullCommands, FC_error };
enum reducedCommand{ RC_move, RC_goBack, RC_dance, RC_numReducedCommands, RC_error };
enum direction{ D_noSet, D_turnDefault,D_moveDefault, D_left, D_right, D_forward, D_backward, D_numDirection, D_noChange };
enum location{ L_noSet, L_wall, L_bookcase, L_hallway, L_default, L_numLocation, L_noChange };

class CommandVector{
    public:
        CommandVector( fullCommand baseType, const char* commandWord );

        //type used for interpretting data
        fullCommand baseType;
        reducedCommand trueType;
        float distance;
        float angle;
        direction dir;
        location loc;
        int repetitions;
        bool aroundSet;

        const char* commandWord;

        friend std::ostream& operator<<( std::ostream& out, CommandVector& command );
};

class SpecialCase{
    public:
        SpecialCase(){}

        virtual bool execute( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ) = 0;
};
#define FLOAT_NOSET -1
#define FLOAT_NOCHANGE -2
#define REP_NOSET -1
#define REP_NOCHANGE -2

class Interpreter{
    public:
        //initializes all shrinking maps
        static void init();

        static bool isInit();

        enum status{ error, numStatus };
    private:
        Interpreter();

        //private CommandAugment Class used to adjust CommandVector values
        class CommandAugment{
            public:
                CommandAugment( SpecialCase* special = NULL );
                CommandAugment( direction dir, location loc, int reps = REP_NOCHANGE, SpecialCase* special = NULL );

            private:
                direction dir;
                location loc;
                int repetitions;
                SpecialCase* special;

            public:
                bool augment( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter );
        };
        //private Conversion class used for cardinal Direct Object Reductions:
        class Conversion{
            public:
                //enumeration specifying the type of conversion
                enum type{ distance, angle, repetition };
                //ctor
                Conversion( Conversion::type cType, float factor );
                Conversion(){}

            private:
                type cType;
                float factor;

            public:
                //converts the value and sets the appropriate parameter for command
                //returns 0 upon failure
                bool convert( float value, CommandVector& command );
        };

        //Our "Shrinking" maps

        //reduces command words to their full command enumeration
        static std::map<std::string, fullCommand> commandMap;
        //reduces command gerund forms to their base command words
        static std::map<std::string, std::string> gerundReductionMap;
        //checks for validity and behavior of adverbs
        static std::map<std::string, CommandAugment> adverbAugmentMap;
        //checks for validity and behavior of location nouns
        static std::map<std::string, CommandAugment> locAugmentMap;
        //checks for validity and behaviour of relative location nouns (left, right, port)
        static std::map<std::string, CommandAugment> relLocAugmentMap;
        //checks for validity and behaviour of conversion factors:
        static std::map<std::string, Conversion> conversionFactorMap;
        //error enable bool
        static std::vector<bool> statusState;
        //simple direct map from full commands to their reduced form
        static reducedCommand commandReduction[FC_numFullCommands];
        //ignored words set (words that dont affect or invalidate commands
        static std::set<std::string> ignoredWords;
        //special adjectives for "in" type preposition
        static std::map<std::string, CommandAugment> qualifiedAdjectives;
        //ignored adjectives
        static std::set<std::string> ignoredAdjectives;


    public:
        //needs to return command msg struct later
        static std::list<CommandVector> interpret( NLP::LinguisticTree& tree );

        static void setStatus( status element, bool value );

        static reducedCommand reduce( fullCommand command );

    private:
        //grabs potential command words from the tree
        static std::list<NLP::LinguisticTree::Iterator> getCommands( NLP::LinguisticTree::Iterator root );
        //breakdown the information
        static void interpretPhraseElements( NLP::LinguisticTree::Iterator& word, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );
        //breakdown helper function
        static void interpretPhrase( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );
        //breakdown for adverb phrases
        static void interpretAdverbPhrase( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );
        //breakdown for interpretting prepositional phrase
        static void interpretPrepositionPhrase( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );
        //breakdown for interpretting a location noun phrase
        static void interpretLocation( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
            bool fromPreposition = 1 );
        //breakdown for interpretting direct object
        static void interpretDirectObject( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );
        //breakdown for cardinal matching pairs
        static bool interpretCardinal( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
            unsigned int startingIndex = 0 );

        static void interpretGerunds( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
            bool isBefore = 0 );

        static void interpretSubClause( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
            bool isBefore = 0 );

        static void interpretBy( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );

        static void interpretIn( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList );

        static bool gerundCommandFind( NLP::LinguisticTree::Iterator& current, std::list<NLP::LinguisticTree::Iterator>& potententialGerund );

        static std::list<NLP::LinguisticTree::Iterator> getGerunds( NLP::LinguisticTree::Iterator phrase );

        //breakdown of individual adverb:
        static bool interpretAdverb( NLP::LinguisticTree::Iterator& word, CommandVector& command, CommandAugment*& augment );

        //returns the associated full command from the command map
        static fullCommand getCooresponding( const char* word );

        //recursive Helper Function;
        static bool depthCommandFind( NLP::LinguisticTree::Iterator& current, std::list<NLP::LinguisticTree::Iterator>& potentialCommands );

        static bool isIgnored( NLP::LinguisticTree::Iterator& wordIter );

        static void invalidate( CommandVector& command );

        static NLP::type getType( NLP::LinguisticTree::Iterator current );

};

//some helpful debugging tools:
const char* getString( fullCommand fc );
const char* getString( reducedCommand rc );
const char* getString( direction d );
const char* getString( location l );

#endif // INTERPRETER_H
