#ifndef INTERPRETER_H
#define INTERPRETER_H

#include "LinguisticTree.h"
#include <list>
#include <map>
#include <set>
#include <string>

enum fullCommand{ FC_move, FC_turn, FC_goLocation, FC_goBack, FC_dance, FC_numFullCommands, FC_error };
enum reducedCommand{ RC_move, RC_goBack, RC_dance, RC_numReducedCommands, RC_error };
enum direction{ D_noSet, D_turnDefault,D_moveDefault, D_left, D_right, D_forward, D_backward, D_numDirection, D_noChange };
enum location{ L_noSet, L_default, L_numLocation, L_noChange };

class CommandVector{
    public:
        CommandVector( fullCommand baseType );

        //type used for interpretting data
        fullCommand baseType;
        reducedCommand trueType;
        float distance;
        float angle;
        direction dir;
        location loc;
        int repetitions;
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
                CommandAugment( direction dir, location loc, float dis = FLOAT_NOCHANGE, float angle = FLOAT_NOCHANGE,
                               int reps = REP_NOCHANGE, SpecialCase* special = NULL );

            private:
                float distance;
                float angle;
                direction dir;
                location loc;
                int repetitions;
                SpecialCase* special;

            public:
                bool augment( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter );
        };

        //Our "Shrinking" maps

        //reduces command words to their full command enumeration
        static std::map<std::string, fullCommand> commandMap;
        //reduces command gerund forms to their full command enumeration
        static std::map<std::string, fullCommand> gerundReductionMap;
        //checks for validity and behavior of adverbs
        static std::map<std::string, CommandAugment> adverbAugmentMap;
        //error enable bool
        static std::vector<bool> statusState;
        //simple direct map from full commands to their reduced form
        static reducedCommand commandReduction[FC_numFullCommands];
        //ignored words set (words that dont affect or invalidate commands
        static std::set<std::string> ignoredWords;


    public:
        //needs to return command msg struct later
        static std::list<CommandVector> interpret( NLP::LinguisticTree& tree );

        static void setStatus( status element, bool value );

        static reducedCommand reduce( fullCommand command );

        static const char* getText( NLP::LinguisticTree::Iterator& wordIter );

    private:
        //grabs potential command words from the tree
        static std::list<NLP::LinguisticTree::Iterator> getCommands( NLP::LinguisticTree::Iterator root );
        //breakdown the information
        static void interpretPhraseElements( NLP::LinguisticTree::Iterator& word, CommandVector& command );
        //breakdown helper function
        static void interpretPhrase( NLP::LinguisticTree::Iterator& phrase, CommandVector& command );
        //breakdown for adverb phrases
        static void interpretAdverb( NLP::LinguisticTree::Iterator& phrase, CommandVector& command );

        //returns the associated full command from the command map
        static fullCommand getCooresponding( const char* word );

        //recursive Helper Function;
        static void depthCommandFind( NLP::LinguisticTree::Iterator& current, std::list<NLP::LinguisticTree::Iterator>& potentialCommands );

        static bool isIgnored( NLP::LinguisticTree::Iterator& wordIter );

        static void invalidate( CommandVector& command );

};

#endif // INTERPRETER_H
