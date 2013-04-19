/**	@name Interpretter.cpp
  *
  *	LinguistTree Interpretter class
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	1.00.00
  *	@date		2013-04-03
  *
  *	Version History:
  *	@version	0.01.01
  *	@date		2013-03-29
  *	File creation
  *	@version	0.01.02
  *	@date		2013-03-30
  *	Implemented Debugging tools for
  * CommandVector
  *	@version	0.01.03
  *	@date		2013-03-30
  *	Added new elements to reduction maps
  * based on our test set.
  *	@version	0.02.01
  *	@date		2013-03-30
  *	Addition of prepositional phrases
  *	@version	0.02.02
  *	@date		2013-04-03
  *	Added Subjective Noun detection and
  * some more error codes. Also added
  * "counterclockwise" to Augment map
  *	@version	1.00.00
  *	@date		2013-04-03
  *	Beta, Added most of Direct object support
  * and added cardinal support
  *	@version	1.01.00
  *	@date		2013-04-15
  *	Gerund support and for verbbal Particle "back"
  * added.
  *	@version	1.01.01
  *	@date		2013-04-18
  *	Added support for "by" preposition
  *	@version	1.01.02
  *	@date		2013-04-18
  *	Added support for "you" in "before" and "after"
  * prepositions.
  *	@version	1.01.03
  *	@date		2013-04-18
  *	Added support for preposition "in"
  * ex. "move in the right direction"
**/
#include "Interpreter.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <sstream>
using namespace std;

//default distance in meters
#define DISTANCE_DEFAULT 1
//default angle of turn in degrees
#define ANGLE_DEFAULT 90

//all of our wonderfully annoying statics
std::map<std::string, fullCommand> Interpreter::commandMap;
std::map<std::string, std::string> Interpreter::gerundReductionMap;
std::map<std::string, Interpreter::CommandAugment> Interpreter::adverbAugmentMap;
std::map<std::string, Interpreter::CommandAugment> Interpreter::locAugmentMap;
std::map<std::string, Interpreter::CommandAugment> Interpreter::relLocAugmentMap;
std::map<std::string, Interpreter::Conversion> Interpreter::conversionFactorMap;
std::vector<bool> Interpreter::statusState;
reducedCommand Interpreter::commandReduction[FC_numFullCommands];
std::set<std::string> Interpreter::ignoredWords;
std::map<std::string, Interpreter::CommandAugment> Interpreter::qualifiedAdjectives;
std::set<std::string> Interpreter::ignoredAdjectives;

//all special case definitions:
//handles "right now" case
class RightSpecial:public SpecialCase{
    protected:
        RightSpecial(){}

    public:
        virtual bool execute( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
            //grab the next word
            NLP::LinguisticTree::Iterator nextWord = wordIter.getNextWord();
            //if the next word doesn't exist or is not "Now" add right as the direction
            //otherwise ignore.
            if( nextWord == nextWord.getEnd() || strcmp( NLP::LinguisticTree::getText(nextWord), "now" ) ){
                //set the direction if not already set
                if( command.dir != D_noSet ){
                    //if it's already been set it's a redundancy
                    return 0;
                }
                command.dir = D_right;
            }
            return 1;
        }

        static SpecialCase* get(){
            return &rightCase;
        }

    private:
        static RightSpecial rightCase;
};
RightSpecial RightSpecial::rightCase;

class ClockwiseSpecial:public SpecialCase{
    protected:
        ClockwiseSpecial(){}

    public:
        virtual bool execute( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
            //dependency of clockwise depends on whether "counter" was placed before it or not.
            NLP::LinguisticTree::Iterator prevWord = wordIter.getPrevWord();
            if( prevWord == prevWord.getEnd() || strcmp( NLP::LinguisticTree::getText(prevWord), "counter" ) ){
                if( command.dir != D_noSet ){
                    //if it's already been set it's a redundancy
                    return 0;
                }
                command.dir = D_right;
            }else{
                if( command.dir != D_noSet ){
                    //if it's already been set it's a redundancy
                    return 0;
                }
                command.dir = D_left;
            }
            return 1;
        }

        static SpecialCase* get(){ return &clockwiseCase; }
    private:
        static ClockwiseSpecial clockwiseCase;
};
ClockwiseSpecial ClockwiseSpecial::clockwiseCase;

class AroundSpecial:public SpecialCase{
    protected:
        AroundSpecial(){}

    public:
        virtual bool execute( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
            if( command.baseType == FC_dance ) return 1;
            if( command.angle != FLOAT_NOSET ){
                return 0;
            }
            command.angle = 180;
            if( command.dir != D_noSet ){
                return 0;
            }
            command.aroundSet = 1;
            return 1;
        }
        static SpecialCase* get(){ return &aroundCase; }

    private:
        static AroundSpecial aroundCase;
};
AroundSpecial AroundSpecial::aroundCase;

class BackSpecial:public SpecialCase{
    protected:
        BackSpecial(){}

    public:
        virtual bool execute( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
            if( command.baseType == FC_dance ) return 1;
            command.baseType = FC_goBack;
            return 0;
        }
        static BackSpecial* get(){ return &backcase; }

    private:
        static BackSpecial backcase;
};
BackSpecial BackSpecial::backcase;

class AdjectiveSpecial:public SpecialCase{
    protected:
        AdjectiveSpecial( const char* word):word(word){
        }

        const char* word;

    public:
        virtual bool execute( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
            //first get the next noun
            NLP::LinguisticTree::Iterator noun = wordIter.getNextWord();
            while( noun != wordIter.getEnd() ){
                if( (*noun)->getType() == NLP::noun ){
                    break;
                }
            }
            if( noun == wordIter.getEnd() ){
                return 0;
            }
            if( word != NULL ){
                if( !strcmp( word, NLP::LinguisticTree::getText( noun ) ) ){
                    return 1;
                }
            }
            return 0;
        }

        static SpecialCase* getDirection(){ return &directionSpecial; }

    private:
        static AdjectiveSpecial directionSpecial;
};

AdjectiveSpecial AdjectiveSpecial::directionSpecial("direction");

//function controls
CommandVector::CommandVector( fullCommand baseType, const char* commandWord ):baseType(baseType),trueType(Interpreter::reduce(baseType)),
    distance(FLOAT_NOSET),angle(FLOAT_NOSET),dir(D_noSet),loc(L_noSet),repetitions(REP_NOSET),aroundSet(0),commandWord(commandWord){}

std::ostream& operator<<( std::ostream& out, CommandVector& command ){
    out<<command.commandWord<<std::endl;
    out<<getString(command.baseType)<<", "<<getString(command.trueType)<<std::endl;
    out<<"Distance: "<<command.distance<<", Angle: "<<command.angle<<std::endl;
    out<<getString(command.dir)<<", "<<getString(command.loc)<<std::endl;
    out<<command.repetitions<<std::endl<<std::endl;
    return out;
}

void Interpreter::init(){
    if( isInit() ) return;
    //command map initialization
    commandMap["turn"] = FC_turn;
    commandMap["rotate"] = FC_turn;
    commandMap["spin"] = FC_turn;
    commandMap["go"] = FC_move;
    commandMap["travel"] = FC_move;
    commandMap["move"] = FC_move;
    commandMap["advance"] = FC_move;
    commandMap["head"] = FC_move;
    commandMap["dance"] = FC_dance;
    commandMap["celebrate"] = FC_dance;
    commandMap["boogie"] = FC_dance;
    commandMap["approach"] = FC_goLocation;
    commandMap["find"] = FC_goLocation;
    commandMap["return"] = FC_goBack;
    //Gerund reduction initialization
    gerundReductionMap["turning"] = "turn";
    gerundReductionMap["rotating"] = "rotate";
    gerundReductionMap["spinning"] = "spin";
    gerundReductionMap["going"] = "go";
    gerundReductionMap["traveling"] = "travel";
    gerundReductionMap["advancing"] = "advance";
    gerundReductionMap["heading"] = "head";
    gerundReductionMap["dancing"] = "dance";
    gerundReductionMap["celebrating"] = "celebrate";
    gerundReductionMap["boogieing"] = "boogie";
    gerundReductionMap["approaching"] = "approach";
    gerundReductionMap["finding"] = "find";
    gerundReductionMap["returning"] = "return";

    //command reduction initialization
    commandReduction[FC_move] = RC_move;
    commandReduction[FC_turn] = RC_move;
    commandReduction[FC_goLocation] = RC_move;
    commandReduction[FC_goBack] = RC_goBack;
    commandReduction[FC_dance] = RC_dance;
    //statusState initialization
    for( int i = 0; i < numStatus; ++i ){
        statusState.push_back(0);
    }

    //adverbAugment initialization
    adverbAugmentMap["right"] = CommandAugment( RightSpecial::get() );
    adverbAugmentMap["left"] = CommandAugment( D_left, L_noChange );
    adverbAugmentMap["forward"] = CommandAugment( D_forward, L_noChange );
    adverbAugmentMap["backwards"] = CommandAugment( D_backward, L_noChange );
    adverbAugmentMap["clockwise"] = CommandAugment( ClockwiseSpecial::get() );
    adverbAugmentMap["counterclockwise"] = CommandAugment( D_left, L_noChange );
    adverbAugmentMap["twice"] = CommandAugment( D_noChange, L_noChange, 2 );
    adverbAugmentMap["thrice"] = CommandAugment( D_noChange, L_noChange, 3 );
    adverbAugmentMap["around"] = CommandAugment( AroundSpecial::get() );
    adverbAugmentMap["ahead"] = CommandAugment( D_forward, L_noChange );
    adverbAugmentMap["back"] = CommandAugment( BackSpecial::get() );
    //locationAugment initialization
    locAugmentMap["Germany"] = CommandAugment( D_noChange, L_wall );
    locAugmentMap["Japan"] = CommandAugment( D_noChange, L_bookcase );
    locAugmentMap["Mexico"] = CommandAugment( D_noChange, L_hallway );
    //relative location augment initialization
    relLocAugmentMap["port"] = CommandAugment( D_left, L_noChange );
    relLocAugmentMap["left"] = CommandAugment( D_left, L_noChange );
    relLocAugmentMap["portside"] = CommandAugment( D_left, L_noChange );
    relLocAugmentMap["right"] = CommandAugment( D_right, L_noChange );
    relLocAugmentMap["starboard"] = CommandAugment( D_right, L_noChange );
    //Conversion Factor initialization
    conversionFactorMap["meters"] = Conversion( Conversion::distance, 1 );
    conversionFactorMap["meter"] = Conversion( Conversion::distance, 1 );
    conversionFactorMap["degree"] = Conversion( Conversion::angle, 1 );
    conversionFactorMap["degrees"] = Conversion( Conversion::angle, 1 );
    conversionFactorMap["time"] = Conversion( Conversion::repetition, 1 );
    conversionFactorMap["times"] = Conversion( Conversion::repetition, 1 );
    conversionFactorMap["feet"] = Conversion( Conversion::distance, .3048 );
    conversionFactorMap["foot"] = Conversion( Conversion::distance, .3048 );
    conversionFactorMap["radian"] = Conversion( Conversion::angle, 180./3.1416 );
    conversionFactorMap["radians"] = Conversion( Conversion::angle, 180./3.1416 );

    //ignored terms
    ignoredWords.insert("now");
    ignoredWords.insert("counter");
    ignoredWords.insert("fast");
    ignoredWords.insert("quickly");
    ignoredWords.insert("rapidly");
    ignoredWords.insert("swiftly");
    ignoredWords.insert("slowly");
    ignoredWords.insert("sluggishly");
    //Adjective initialization;
    qualifiedAdjectives["right"] = CommandAugment( D_right, L_noChange, REP_NOCHANGE, AdjectiveSpecial::getDirection() );
    qualifiedAdjectives["left"] = CommandAugment( D_left, L_noChange, REP_NOCHANGE, AdjectiveSpecial::getDirection() );
    qualifiedAdjectives["forward"] = CommandAugment( D_forward, L_noChange, REP_NOCHANGE, AdjectiveSpecial::getDirection() );
    qualifiedAdjectives["backward"] = CommandAugment( D_backward, L_noChange, REP_NOCHANGE, AdjectiveSpecial::getDirection() );

}

bool Interpreter::isInit(){
    return statusState.size();
}

void Interpreter::setStatus( Interpreter::status element, bool value ){
    if( !isInit() ) init();
    statusState[element] = value;
}

reducedCommand Interpreter::reduce( fullCommand command ){
    if( !isInit() ) return RC_error;
    if( command >= FC_numFullCommands ) return RC_error;
    return commandReduction[command];
}

std::list<CommandVector> Interpreter::interpret( NLP::LinguisticTree& tree ){
    if( !isInit() ) init();
    //grab the commands
    std::list<NLP::LinguisticTree::Iterator> commandWordList = getCommands( tree.begin() );
    //grab the "go" term from "go and"
    NLP::LinguisticTree::Iterator goToken = commandWordList.front();
    //remove that term from the list
    commandWordList.pop_front();

    std::list<CommandVector> commandList;
    if( commandWordList.empty() ){
        if( statusState[error] ){
            cerr<<"No command found"<<endl;
        }
        return commandList;
    }

    //transform commandWords to appropriate command defaults and removes invalid command words
    std::list<NLP::LinguisticTree::Iterator>::iterator currentWord = commandWordList.begin();
    while( currentWord != commandWordList.end() ){
        NLP::Word* word = (NLP::Word*) (*(*currentWord));
        //grab the command enumeration from the text form of the word
        fullCommand commandType = getCooresponding( word->getText() );
        //if the word is unrecognized
        if( commandType == FC_error ){
            std::list<NLP::LinguisticTree::Iterator>::iterator toBeDeleted = currentWord;
            currentWord++;
            //erase the word from the command list
            commandWordList.erase( toBeDeleted );
            //out the error if errors enabled
            if( statusState[error] ){
                cerr<<"Unrecognized Command Verb \""<<word->getText()<<"\". Discarding Command."<<endl;
            }
            continue;
        }
        commandList.push_back( CommandVector(commandType, word->getText() ) );
        currentWord++;
    }

    //alright here comes the hard part, currently only basic case is being considered
    //TODO: handles second type and "Rogue Words"
    std::list<NLP::LinguisticTree::Iterator>::iterator current = commandWordList.begin();
    std::list<CommandVector>::iterator command = commandList.begin();
    while( current != commandWordList.end() ){
        //get temporary copies
        std::list<NLP::LinguisticTree::Iterator>::iterator temp = current;
        std::list<CommandVector>::iterator tempCommand = command;
        //update iterators
        current++;
        command++;
        //interpret the phrase elements
        interpretPhraseElements( *temp, tempCommand, commandList );
        //if, for any reason the command was invalidated remove from list
        if( tempCommand->baseType == FC_error ){
            if( statusState[error] ){
                cerr<<"Removing Command."<<endl;
            }
            commandWordList.erase(temp);
            commandList.erase(tempCommand);
        }
    }
    //Invalidate commands based on arguments
    command = commandList.begin();
    while( command != commandList.end() ){
        list<CommandVector>::iterator currentCommand = command;
        command++;

        bool invalid = 0;

        switch( currentCommand->baseType ){
            case FC_move:
                //if location is set both distance and angle are invalid
                //ex. "move 5 degrees to the chair" "move 5 meters to the chair"
                //"move left to the chair"
                //althought the second command might be valid, it is currently not supported
                if( currentCommand->loc != L_noSet ){
                    if( currentCommand->distance != FLOAT_NOSET || currentCommand->angle != FLOAT_NOSET ){
                        invalid = 1;
                    }
                    if( currentCommand->dir != D_noSet ){
                        invalid = 1;
                    }
                }
                //if angle and distance are set it is invalid
                //ex. "move 40 degrees 5 meters"
                if( currentCommand->distance != FLOAT_NOSET && currentCommand->angle != FLOAT_NOSET ){
                    invalid = 1;
                }
                // if direction is set to anthing but left or right, angle should be zero
                //ex. "move 45 degrees forward"
                if( currentCommand->dir != D_noSet && currentCommand->dir != D_right && currentCommand->dir != D_left){
                    if( currentCommand->angle != FLOAT_NOSET ){
                        invalid = 1;
                    }
                }
                //TODO: check for more cases
                break;
            case FC_turn:
                //if distance was set in a turn command it is invalidated:
                //ex. "Turn left 5 meters"
                if( currentCommand->distance != FLOAT_NOSET ){
                    invalid = 1;
                }
                //location and angle are set, the command is invalidated:
                //ex. "turn 50 degrees to the chair"
                //technically a valid command but we do not support it.
                if( currentCommand->angle != FLOAT_NOSET && currentCommand->loc != L_noSet){
                    invalid = 1;
                }
                //if direction is set to anything other than left or right
                //ex. "turn forward 12 degrees"
                if( currentCommand->dir != D_noSet ){
                    if( currentCommand->dir != D_left && currentCommand->dir != D_right && currentCommand->dir != D_turnDefault ){
                        invalid = 1;
                    }
                }
                break;
            case FC_goLocation:
                //for go location it's simple, anything other than location is invalid
                //and if it doesn't have a location it's invalid
                //ex. "find the chair 40 degrees"
                //ex. "find 4 meters"
                //ex. "find forward"
                if( currentCommand->loc == L_noSet ){
                    invalid = 1;
                }
                if( currentCommand->angle != FLOAT_NOSET || currentCommand->distance != FLOAT_NOSET ){
                    invalid = 1;
                }
                if( currentCommand->dir != D_noSet ){
                    invalid = 1;
                }
                break;
            case FC_goBack:
                //for now goBack works like goLocation
                //except location doesn't need to be set
                if( currentCommand->angle != FLOAT_NOSET || currentCommand->distance != FLOAT_NOSET ){
                    invalid = 1;
                }
                if( currentCommand->dir != D_noSet ){
                    invalid = 1;
                }
                break;
            case FC_dance:
                //no arguments should be set for dance
                //for now
                if( currentCommand->loc != L_noSet ){
                    invalid = 1;
                }
                if( currentCommand->angle != FLOAT_NOSET || currentCommand->distance != FLOAT_NOSET ){
                    invalid = 1;
                }
                if( currentCommand->dir != D_noSet ){
                    invalid = 1;
                }
                break;
            default:
                //unsupported command type
                invalid = 1;
                break;
        }

        //if it was invalidated remove it
        if( invalid ){
            if( statusState[error] ){
                cerr<<"Invalid command due to improper parameters for command word."<<endl;
            }
            commandList.erase( currentCommand );
        }
    }
    //first rogue word handler:
    NLP::LinguisticTree::Iterator subClause = tree.begin();
    while( subClause != tree.end() ){
        subClause = subClause.getLastChild();
        if( getType( subClause ) == NLP::subClause ){
            break;
        }
    }
    if( !commandList.empty() ){
        if( getType( subClause ) == NLP::subClause ){
            list<CommandVector>::iterator lastCommand = commandList.end();
            lastCommand--;
            interpretPrepositionPhrase( subClause, lastCommand, commandList );
            if( lastCommand->baseType == FC_error ){
                commandList.erase( lastCommand );
            }
        }
    }
    //set defaults for commands that are not invalidated
    for( command = commandList.begin(); command != commandList.end(); command++ ){
        switch( command->baseType ){
            case FC_move:
                //if no direction set, set it
                if( command->dir == D_noSet ){
                    command->dir = D_moveDefault;
                }
                //if this move does not have a location
                if( command->loc == L_noSet ){
                    //if it is turn type default the angle
                    if( command->dir == D_left || command->dir == D_right ){
                        command->angle = ANGLE_DEFAULT;
                    }
                    //also default the distance
                    if( command->distance == FLOAT_NOSET ){
                        command->distance = DISTANCE_DEFAULT;
                    }
                }
                break;
            case FC_turn:
                //if no direction set, set it
                if( command->dir == D_noSet ){
                    command->dir = D_turnDefault;
                }
                //if this does not have a location or angle
                if( command->loc == L_noSet && command->angle == FLOAT_NOSET ){
                    // default the angle
                    command->angle = ANGLE_DEFAULT;
                }
                break;
            case FC_goLocation:
                //if no location assume default
                //@note should not happen
                if( command->loc == L_noSet ){
                    command->loc = L_default;
                }
                break;
            //no special cases for go back or dance
            default:
                break;
        }
        if( command->repetitions == REP_NOSET ){
            command->repetitions = 1;
        }
    }
    return commandList;
}

std::list<NLP::LinguisticTree::Iterator> Interpreter::getCommands( NLP::LinguisticTree::Iterator root ){
    std::list<NLP::LinguisticTree::Iterator> potentialCommandWordList;
    //recursive call
    for( unsigned int i = 0; i < root.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = root.getChild(i);
        if( getType(child) == NLP::sentence || getType(child) == NLP::verbPhrase || getType(child) == NLP::sinv ){
            if( !depthCommandFind( child, potentialCommandWordList ) ){
                return potentialCommandWordList;
            }
        }
    }
    return potentialCommandWordList;
}

fullCommand Interpreter::getCooresponding( const char* word ){
    try{
        return commandMap.at( word );
    }catch( std::out_of_range oor ){
        return FC_error;
    }
    return FC_error;
}

bool Interpreter::depthCommandFind( NLP::LinguisticTree::Iterator& current, std::list<NLP::LinguisticTree::Iterator>& potentialCommands ){
    for( unsigned int i = 0; i < current.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = current.getChild(i);
        NLP::LinguisticTree::Iterator noun;
        switch( getType(child) ){
            case NLP::verbBase:
            case NLP::verbPresent:
                potentialCommands.push_back( child );
                break;
            case NLP::verbPhrase:
            //DEBUG: currently testing this blocker out
            //case NLP::sentence:
                depthCommandFind( child, potentialCommands );
                break;
            //DEBUG: curently testing this new blocker out
            case NLP::nounPhrase:
                //if we've found a subjective noun
                noun = child.getFirstChild();
                cout<<NLP::LinguisticTree::getText(noun)<<endl;
                if( getType(current) == NLP::sentence && strcmp( NLP::LinguisticTree::getText(noun), "you" ) ){
                    //we know there are no commands
                    if( statusState[error] ){
                        cerr<<"Subjective noun found, sentence is not a command"<<endl;
                    }
                    return 0;
                }
                break;
            default:
                break;
        }
    }
    return 1;
}

void Interpreter::interpretPhraseElements( NLP::LinguisticTree::Iterator& word, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    //grab the parent verb phrase
    NLP::LinguisticTree::Iterator directParent = word.getParent();
    //Go through all the phrases
    for( unsigned int i = 0; i < directParent.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator currentPhrase = directParent.getChild(i);
        if( currentPhrase == word ) continue;
        interpretPhrase( currentPhrase, command, commandList );
    }
}

void Interpreter::interpretPhrase( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    switch( getType(phrase) ){
        //interpret adverbs and particles
        case NLP::advPhrase:
        case NLP::phrasalVerbPart:
        //just in case of misclassification
        case NLP::adjPhrase:
        case NLP::verbPhrase:
            interpretAdverbPhrase( phrase, command, commandList );
            break;
        case NLP::prepPhrase:
            interpretPrepositionPhrase( phrase, command, commandList );
            break;
        case NLP::nounPhrase:
            interpretDirectObject( phrase, command, commandList );
            break;
        default:
            break;
    }
}

void Interpreter::interpretAdverbPhrase( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    for( unsigned int i = 0; i < phrase.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = phrase.getChild(i);
        CommandAugment* currentAugment = NULL;
        switch( getType(child) ){
            case NLP::adverb:
            case NLP::particle:
            //just in case of bad classification
            case NLP::adjective:
            case NLP::verbPastPart:
            case NLP::preposition:
                if( !interpretAdverb( child, *command, currentAugment ) ){
                    return;
                }
                break;
            //if nested needs to be evaluated:
            case NLP::advPhrase:
            case NLP::adjPhrase:
                interpretAdverbPhrase( child, command, commandList );
            case NLP::nounPhrase:
                interpretDirectObject( child, command, commandList );
            case NLP::prepPhrase:
                interpretPrepositionPhrase( child, command, commandList );
            default:
                //TODO: handle unrecognized types
                break;
        }
        //check to see if we caught an Augment
        if( currentAugment != NULL ){
            //check to make sure it executes:
            if( !currentAugment->augment( *command, child ) ){
                //if it fails the command is erroneous:
                if( statusState[error] ){
                    cerr<<"Augmentation failed. Parameter redundancy detected."<<endl;
                }
                invalidate( *command );
                return;
            }
        }
    }
}

void Interpreter::interpretPrepositionPhrase( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    //first child is always the preposition:
    //TODO: Try to find a counter-example
    NLP::LinguisticTree::Iterator child = phrase.getFirstChild();
    const char* preposition = NLP::LinguisticTree::getText(child);
    //want to replace if else block with a map later:
    if( !strcmp( "to", preposition ) ){
        //if it is a to then we have a location
        if( command->baseType == FC_goLocation ){
            //if the command is a direct object location type like "find" or "approach"
            //the command is invalidated i.e. "find to the chair" doesn't make sense.
            if( statusState[error] ){
                cerr<<"invalid preposition \"to\" found in command of type GoTo"<<endl;
            }
            invalidate( *command);
            return;
        }
        child = child.getNextSibling();
        interpretLocation( child, command, commandList );
    }else if( !strcmp( "after", preposition ) ){
        NLP::LinguisticTree::Iterator nextWord = child.getNextWord();
        child = child.getNextSibling();
        if( !strcmp( "you", NLP::LinguisticTree::getText( nextWord ) ) ){
            interpretSubClause( child, command, commandList, 0 );
        }else{
            interpretGerunds( child, command, commandList, 0 );
        }
    }else if( !strcmp( "before", preposition ) ){
        child = child.getNextSibling();
        NLP::LinguisticTree::Iterator nextWord = child.getNextWord();
        if( !strcmp( "you", NLP::LinguisticTree::getText( nextWord ) ) ){
            interpretSubClause( child, command, commandList, 1 );
        }else{
            interpretGerunds( child, command, commandList, 1 );
        }
    }else if( !strcmp("by", preposition ) ){
        child = child.getNextSibling();
        interpretBy( child, command, commandList );
    }else if( !strcmp("in", preposition ) ){
        child = child.getNextSibling();
        interpretIn( child, command, commandList );
    }else{
        invalidate( *command );
        if( statusState[error] ){
            cerr<<"Unrecognized preposition \""<<preposition<<"\". Invalidating Command"<<endl;
        }
    }
}

void Interpreter::interpretLocation( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
    bool fromPreposition ){

    if( getType(phrase) != NLP::nounPhrase ){
        invalidate( *command );
        if( statusState[error] ){
            cerr<<"Attempt to gather location from non-type Noun Phrase. Invalidating Command."<<endl;
        }
        return;
    }
    for( unsigned int i = 0; i < phrase.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = phrase.getChild(i);
        CommandAugment* currentAugment = NULL;
        switch( getType(child) ){
            case NLP::determiner:
                break;
            case NLP::noun:
            case NLP::pluralNoun:
            //possible misclassifications
            case NLP::adjective:

                try{
                    //if it is an location grab it's augment
                    currentAugment = &locAugmentMap.at( NLP::LinguisticTree::getText(child) );
                }catch( std::out_of_range oor ){
                    //otherwise if it is a relative location grab it's augment if it is consider
                    if( fromPreposition ){
                        try{
                            currentAugment = &relLocAugmentMap.at( NLP::LinguisticTree::getText(child) );
                        }catch( std::out_of_range ){}
                    }
                    //otherwise check to see if we can ignore it
                    if( !isIgnored( child ) && currentAugment == NULL ){
                        //if we can't it's an erroneous command
                        if( statusState[error] ){
                            cerr<<"Invalidating location noun \""<<NLP::LinguisticTree::getText(child)<<"\". Invalidating Command."<<endl;
                        }
                        invalidate( *command );
                        return;
                    }
                }
                break;
            //in case of nesting
            case NLP::nounPhrase:
            //or phrase misclassification
            case NLP::adjPhrase:
                interpretAdverbPhrase( child, command, commandList );
                break;
            case NLP::cardinalNum:
                //accidental inclusion of cardinal
                if( !interpretCardinal( phrase, command, commandList, i ) ){
                    invalidate( *command );
                    return;
                }else{
                    //since phrase was finished by interpretCardinal
                    return;
                }
                break;
            default:
                if( !isIgnored( child ) ){
                    if( statusState[error] ){
                        cerr<<"Warning: Unsupported type "<<getType(child)<<" in location interpretation."<<endl;
                    }
                    invalidate( *command );
                    return;
                }
                break;
        }
        if( currentAugment != NULL ){
            //check to make sure it executes:
            if( !currentAugment->augment( *command, child ) ){
                //if it fails the command is erroneous:
                if( statusState[error] ){
                    cerr<<"Augmentation failed. Parameter redundancy detected."<<endl;
                }
                invalidate( *command );
                return;
            }
        }
    }

}

void Interpreter::interpretDirectObject( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    //behaviour changes based on full type
    if( command->baseType == FC_goLocation ){
        interpretLocation( phrase, command, commandList, 0  );
    }else{
        //if it's not a cardinal for non goto
        if( !interpretCardinal( phrase, command, commandList ) ){
            //for now invalidate, but there is still some work to be done
            //for ex. "go home", "return home", "head home", but "move home" doesn't work
            invalidate( *command );
        }
    }
}

bool Interpreter::interpretCardinal( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
    unsigned int startingIndex ){

    bool isSet = 0;
    float value = FLOAT_NOSET;
    //helper for pulling floating point from text
    istringstream valueStream;

    for( unsigned int i = startingIndex; i < phrase.getNumChildren(); i++ ){
        NLP::LinguisticTree::Iterator child = phrase.getChild( i );
        CommandAugment* currentAugment = NULL;
        Conversion* currentConversion = NULL;
        switch( getType( child ) ){
            //just incase an adverb or adjective "fell" into the noun phrase
            case NLP::adverb:
            case NLP::adjective:
                if( !interpretAdverb( child, *command, currentAugment ) ){
                   return 0;
                }
                break;
            //finding the cardinal number
            case NLP::cardinalNum:
                //if the value was already set
                if( value != FLOAT_NOSET ){
                    return 0;
                }
                //load the float
                valueStream.str( NLP::LinguisticTree::getText( child ) );
                valueStream>>value;
                //if the float couldn't be loaded
                if( valueStream.bad() || value < 0 ){
                    if(statusState[error]){
                        cerr<<"Couldn't recognize number"<<endl;
                    }
                    return 0;
                }
                break;
            //handlers for noun phrases
            case NLP::noun:
            case NLP::pluralNoun:
                //check to see for conversion
                try{
                    currentConversion = &conversionFactorMap.at( NLP::LinguisticTree::getText(child) );
                }catch( std::out_of_range oor ){
                    //if it is not a conversion check to see if it is a misclassified adverb
                    //ex. "rotate right"
                    if( !interpretAdverb( child, *command, currentAugment ) ){
                        return 0;
                    }
                }
                //if we found a conversion
                if( currentConversion != NULL ){
                    //but we have no value to convert
                    if( value == FLOAT_NOSET ){
                        if( statusState[error] ){
                            cerr<<"Tried to Convert without Value"<<endl;
                        }
                        return 0;
                    }
                    //otherwise convert
                    if( !currentConversion->convert( value, *command ) ){
                        if( statusState[error] ){
                            cerr<<"Invalid Conversion attempt due to redundancy"<<endl;
                        }
                        return 0;
                    }
                    isSet = 1;
                }
                break;
            //nested cardinal phrase, rare but can happen
            //example: "move 2 meters 3 times"
            case NLP::nounPhrase:
                //if interpretation failed.
                if( !interpretCardinal( child, command, commandList ) ){
                    return 0;
                }
                break;

            default:
                if( !isIgnored( child ) ){
                    if( statusState[error] ){
                        cerr<<"Warning: Unsupported type "<<getType(child)<<" in cardinal interpretation."<<endl;
                    }
                    return 0;
                }
        }
        if( currentAugment != NULL ){
            //check to make sure it executes:
            if( !currentAugment->augment( *command, child ) ){
                //if it fails the command is erroneous:
                if( statusState[error] ){
                    cerr<<"Augmentation failed. Parameter redundancy detected."<<endl;
                }
                return 0;
            }
        }
    }
    //if a conversion was not found but a number was i.e. "move 3"
    if( value != FLOAT_NOSET && isSet == 0 ){
        if( statusState[error] ){
            cerr<<"Number found, but was not assigned."<<endl;
        }
        return 0;
    }
    return 1;
}

void Interpreter::interpretGerunds( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command,
    std::list<CommandVector>& commandList, bool isBefore ){

    //This should all look familiar,
    //first grab the potential gerund command words
    std::list<NLP::LinguisticTree::Iterator> gerundWordList = getGerunds( phrase );

    if( gerundWordList.empty() ){
        if( statusState[error] ){
            cerr<<"Expected Gerund in PrepositionalPhrase, invalidating Command";
        }
        invalidate( *command );
        return;
    }

    //create a commandList for the gerunds
    std::list<CommandVector> gerundList;
    for( std::list<NLP::LinguisticTree::Iterator>::iterator i = gerundWordList.begin();
        i != gerundWordList.end(); ++i ){
        std::string word;
        fullCommand fc_type;

        try{
            word = gerundReductionMap.at( NLP::LinguisticTree::getText(*i) );
            fc_type = commandMap.at(word);
        }catch( std::out_of_range oor ){
            if( statusState[error] ){
                cerr<< "Invalid gerund \""<<NLP::LinguisticTree::getText(*i)<<
                    "\" found in preposition phrase, invalidating command"<<endl;
            }
            invalidate( *command );
            return;
        }
        gerundList.push_back( CommandVector( fc_type, word.c_str() ) );
    }

    std::list<NLP::LinguisticTree::Iterator>::iterator current = gerundWordList.begin();
    std::list<CommandVector>::iterator gerund = gerundList.begin();
    while( current != gerundWordList.end() ){
        //get temporary copies
        std::list<NLP::LinguisticTree::Iterator>::iterator temp = current;
        std::list<CommandVector>::iterator tempCommand = gerund;
        //update iterators
        current++;
        gerund++;
        //interpret the phrase elements
        interpretPhraseElements( *temp, tempCommand, gerundList );
        //if, for any reason the command was invalidated remove from list
        if( tempCommand->baseType == FC_error ){
            if( statusState[error] ){
                cerr<<"Removing Gerund."<<endl;
            }
            gerundWordList.erase(temp);
            gerundList.erase(tempCommand);

            //TODO::Potentially invalidate the entire command.
        }
    }

    if( !isBefore ){
        commandList.insert( command, gerundList.begin(), gerundList.end() );
    }else{
        command++;
        commandList.insert( command, gerundList.begin(), gerundList.end() );
        command--;

    }
}

void Interpreter::interpretSubClause( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList,
    bool isBefore ){

    //This should all look familiar,
    //first grab the potential command words
    std::list<NLP::LinguisticTree::Iterator> partCommandWordList = getCommands( phrase );

    if( partCommandWordList.empty() ){
        if( statusState[error] ){
            cerr<<"Expected Commands in PrepositionalPhrase, invalidating Command";
        }
        invalidate( *command );
        return;
    }

    //create a commandList for the gerunds
    std::list<CommandVector> partCommandList;
    for( std::list<NLP::LinguisticTree::Iterator>::iterator i = partCommandWordList.begin();
        i != partCommandWordList.end(); ++i ){
        fullCommand fc_type;

        try{
            fc_type = commandMap.at( NLP::LinguisticTree::getText( *i ) );
        }catch( std::out_of_range oor ){
            if( statusState[error] ){
                cerr<< "Invalid command \""<<NLP::LinguisticTree::getText(*i)<<
                    "\" found in preposition phrase, invalidating command"<<endl;
            }
            invalidate( *command );
            return;
        }
        partCommandList.push_back( CommandVector( fc_type, NLP::LinguisticTree::getText( *i ) ) );
    }

    std::list<NLP::LinguisticTree::Iterator>::iterator current = partCommandWordList.begin();
    std::list<CommandVector>::iterator currentCommand = partCommandList.begin();
    while( current != partCommandWordList.end() ){
        //get temporary copies
        std::list<NLP::LinguisticTree::Iterator>::iterator temp = current;
        std::list<CommandVector>::iterator tempCommand = currentCommand;
        //update iterators
        current++;
        currentCommand++;
        //interpret the phrase elements
        interpretPhraseElements( *temp, tempCommand, partCommandList );
        //if, for any reason the command was invalidated remove from list
        if( tempCommand->baseType == FC_error ){
            if( statusState[error] ){
                cerr<<"Removing Gerund."<<endl;
            }
            partCommandWordList.erase(temp);
            partCommandList.erase(tempCommand);

            //TODO::Potentially invalidate the entire command.
        }
    }

    if( !isBefore ){
        commandList.insert( command, partCommandList.begin(), partCommandList.end() );
    }else{
        command++;
        commandList.insert( command, partCommandList.begin(), partCommandList.end() );
        command--;

    }

}

void Interpreter::interpretBy( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    std::list<NLP::LinguisticTree::Iterator> potentialGerund = getGerunds( phrase );
    if( potentialGerund.size() != 1 ){
        invalidate( *command );
        if( statusState[error] ){
            cerr<<"Expected only one gerund in \"by\" position"<<endl;
        }
        return;
    }
    fullCommand commandType = FC_error;
    try{
        commandType = commandMap.at( gerundReductionMap.at( NLP::LinguisticTree::getText( potentialGerund.back() ) ) );
    }catch( std::out_of_range oor ){
        if( statusState[error] ){
            cerr<<"Unrecognized gerund in \"by\" position. Invalidating command"<<endl;
        }
        invalidate(*command);
    }
    if( commandType != FC_move && commandType != FC_turn ){
        cerr<<"Unrecognized gerund augment in \"by\" position"<<endl;
        invalidate( *command );
    }
    command->aroundSet = 0;
    interpretPhraseElements( potentialGerund.back(), command, commandList );
}

std::list<NLP::LinguisticTree::Iterator> Interpreter::getGerunds( NLP::LinguisticTree::Iterator phrase ){
    std::list<NLP::LinguisticTree::Iterator> potentialGerundList;
    //if an invalid iterator was given:
    if( *phrase == NULL ) return potentialGerundList;
    if( (*phrase)->getType() != NLP::sentence ) return potentialGerundList;
    gerundCommandFind( phrase, potentialGerundList );
    return potentialGerundList;
}

bool Interpreter::gerundCommandFind( NLP::LinguisticTree::Iterator& current, std::list<NLP::LinguisticTree::Iterator>& potententialGerund ){
    for( unsigned int i = 0 ; i < current.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = current.getChild(i);
        switch( getType(child) ){
            case NLP::verbGerund:
                potententialGerund.push_back(child);
                break;
            case NLP::verbPhrase:
                if( !gerundCommandFind( child, potententialGerund ) )
                    return 0;
                break;
            case NLP::nounPhrase:
                //always look for subjective nouns
                if( getType( current ) == NLP::sentence ){
                    if( statusState[error] ){
                        cerr<<"Subjective noun found in prepositional phrase, error"<<endl;
                    }
                    potententialGerund.clear();
                    return 0;
                }
                break;
            default:
                break;
        }
    }
    return 1;
}

void Interpreter::interpretIn( NLP::LinguisticTree::Iterator& phrase, std::list<CommandVector>::iterator command, std::list<CommandVector>& commandList ){
    //alright so we should be in the noun phrase.
    for( unsigned int i = 0; i < phrase.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = phrase.getChild(i);
        CommandAugment* augment = NULL;
        switch( getType(child ) ){
            case NLP::determiner:
                break;
            case NLP::adjective:
                try{
                    augment = &qualifiedAdjectives.at( NLP::LinguisticTree::getText(child) );
                }catch( std::out_of_range oor ){
                    if( statusState[error] ){
                        cerr<<"unrecognized adjective \""<<NLP::LinguisticTree::getText( child )<<"\" inside preposition \"in\""<<endl;
                    }
                    invalidate( *command );
                    return;
                }
                break;
            case NLP::noun:
                if( strcmp( NLP::LinguisticTree::getText(child), "direction" ) ){
                    if( statusState[error] ){
                        cerr<<"unrecognized noun \""<<NLP::LinguisticTree::getText( child )<<"\" inside preposition \"in\""<<endl;
                    }
                    invalidate( *command );
                    return;
                }
                break;
            default:
                break;
        }
        if( augment != NULL ){
            if( !augment->augment( *command, child ) ){
                if( statusState[error] ){
                    cerr<<"Augmentation of \""<<NLP::LinguisticTree::getText( child )<<"\" failed inside preposition \"in\""<<endl;
                }
                invalidate( *command );
                return;
            }
        }
    }
}

bool Interpreter::interpretAdverb( NLP::LinguisticTree::Iterator& word, CommandVector& command, CommandAugment*& augment ){
    try{
        //if it is an augmentation adverb grab it's augment
        augment = &adverbAugmentMap.at( NLP::LinguisticTree::getText(word) );
    }catch( std::out_of_range oor ){
        //otherwise check to see if we can ignore it
        if( !isIgnored( word ) ){
            //if we can't it's an erroneous command
            if( statusState[error] ){
                cerr<<"Invalidating adverb \""<<NLP::LinguisticTree::getText(word)<<"\". Invalidating Command."<<endl;
            }
            invalidate( command );
            return 0;
        }
    }
    return 1;
}

bool Interpreter::isIgnored( NLP::LinguisticTree::Iterator& wordIter ){
    return ignoredWords.find( NLP::LinguisticTree::getText(wordIter) ) != ignoredWords.end();
}

void Interpreter::invalidate( CommandVector& command ){
    command.baseType = FC_error;
    command.trueType = RC_error;
}

Interpreter::CommandAugment::CommandAugment( SpecialCase* special ):dir( D_noChange ), loc( L_noChange ),
    repetitions(REP_NOCHANGE),special(special){}

Interpreter::CommandAugment::CommandAugment( direction dir, location loc, int reps, SpecialCase* special ):
    dir(dir),loc(loc),repetitions(reps),special(special){}

bool Interpreter::CommandAugment::augment( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
    if( special != NULL ){
        if( !special->execute( command, wordIter ) ){
            return false;
        }
    }
    //if the direction is changed
    if( dir != D_noChange  ){
        //same as before
        if( command.dir != D_noSet || command.aroundSet ){
            return false;
        }
        command.dir = dir;
    }
    //if the location is changed
    if( loc != L_noChange ){
        //same as before
        if( command.loc != L_noSet ){
            return false;
        }
        command.loc = loc;
    }
    if( repetitions != REP_NOCHANGE ){
        if( command.repetitions != REP_NOSET ){
            return false;
        }
        command.repetitions = repetitions;
    }
    return true;
}

Interpreter::Conversion::Conversion( Interpreter::Conversion::type cType, float factor ):cType(cType), factor(factor){}

//TODO: SET UP FOR THINGS SUCH AS "2 feet 5 inches"
//probably need to store last conversion word inside command for check
bool Interpreter::Conversion::convert( float value, CommandVector& command ){
    //based on the type of conversion it is
    //check to see if the value is already set
    //if so return failure.
    //otherwise set it.
    switch (cType){
        case distance:
            if( command.distance != FLOAT_NOSET ){
                return 0;
            }
            command.distance = value*factor;
            break;
        case angle:
            if( command.angle != FLOAT_NOSET ){
                return 0;
            }
            command.angle = value*factor;
            break;
        case repetition:
            if( command.repetitions != REP_NOSET ){
                return 0;
            }
            command.repetitions = (int) value*factor;
            //check to make sure repetitions was a whole number
            //no "move 1.5 times
            if( command.repetitions != value*factor ){
                return 0;
            }
            break;
    }
    return 1;
}

NLP::type Interpreter::getType( NLP::LinguisticTree::Iterator current ){
    if ( *current == NULL ){
        return NLP::unrecognizedType;
    }else{
        return (*current)->getType();
    }
}

const char* getString( fullCommand fc ){
    switch( fc ){
        case FC_move:
            return "Move";
        case FC_turn:
            return "Turn";
        case FC_goBack:
            return "Return";
        case FC_goLocation:
            return "GoTo";
        case FC_dance:
            return "Dance";
        case FC_error:
            return "ERROR";
        default:
            break;
    }
    return "";
}

const char* getString( reducedCommand rc ){
    switch( rc ){
        case RC_move:
            return "Move";
        case RC_goBack:
            return "Return";
        case RC_dance:
            return "Dance";
        case RC_error:
            return "ERROR";
        default:
            break;
    }
    return "";
}

const char* getString( direction d ){
    switch( d ){
        case D_noSet:
            return "Not Set";
        case D_moveDefault:
        case D_turnDefault:
            return "Default";
        case D_forward:
            return "Forward";
        case D_backward:
            return "Backward";
        case D_left:
            return "Left";
        case D_right:
            return "Right";
        default:
            break;
    }
    return "";
}

const char* getString( location l ){
    switch( l ){
        case L_noSet:
            return "Not Set";
        //DEBUG: for testing only
        case L_wall:
            return "Wall";
        case L_bookcase:
            return "Bookcase";
        case L_hallway:
            return "Hallway";
        case L_default:
            return "Default";
        default:
            break;
    }
    return "";
}
