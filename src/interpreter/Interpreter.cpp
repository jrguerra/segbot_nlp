/**	@name Interpretter.cpp
  *
  *	LinguistTree Interpretter class
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	0.01.03
  *	@date		2013-03-30
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
  *
**/
#include "Interpreter.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
using namespace std;

//default distance in meters
#define DISTANCE_DEFAULT 1
//default angle of turn in degrees
#define ANGLE_DEFAULT 90

//all of our wonderfully annoying statics
std::map<std::string, fullCommand> Interpreter::commandMap;
static std::map<std::string, fullCommand> gerundReductionMap;
std::map<std::string, Interpreter::CommandAugment> Interpreter::adverbAugmentMap;
std::map<std::string, Interpreter::CommandAugment> Interpreter::locAugmentMap;
std::vector<bool> Interpreter::statusState;
reducedCommand Interpreter::commandReduction[FC_numFullCommands];
std::set<std::string> Interpreter::ignoredWords;

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
            command.dir = D_turnDefault;
            return 1;
        }
        static SpecialCase* get(){ return &aroundCase; }

    private:
        static AroundSpecial aroundCase;
};
AroundSpecial AroundSpecial::aroundCase;

//function controls
CommandVector::CommandVector( fullCommand baseType ):baseType(baseType),trueType(Interpreter::reduce(baseType)),
    distance(FLOAT_NOSET),angle(FLOAT_NOSET),dir(D_noSet),loc(L_noSet),repetitions(REP_NOSET){}

std::ostream& operator<<( std::ostream& out, CommandVector& command ){
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
    commandMap["dance"] = FC_dance;
    commandMap["celebrate"] = FC_dance;
    commandMap["boogie"] = FC_dance;
    commandMap["approach"] = FC_goLocation;
    commandMap["find"] = FC_goLocation;
    commandMap["return"] = FC_goBack;

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
    adverbAugmentMap["twice"] = CommandAugment( D_noChange, L_noChange, 2 );
    adverbAugmentMap["thrice"] = CommandAugment( D_noChange, L_noChange, 3 );
    adverbAugmentMap["around"] = CommandAugment( AroundSpecial::get() );
    //locationAugment initialization
    locAugmentMap["port"] = CommandAugment( D_left, L_noChange );
    locAugmentMap["left"] = CommandAugment( D_left, L_noChange );
    locAugmentMap["portside"] = CommandAugment( D_left, L_noChange );
    locAugmentMap["right"] = CommandAugment( D_right, L_noChange );
    locAugmentMap["starboard"] = CommandAugment( D_right, L_noChange );
    //DEBUG: REMOVE WHEN DONE TESTING
    locAugmentMap["chair"] = CommandAugment( D_noChange, L_chair );

    //ignored terms
    ignoredWords.insert("now");
    ignoredWords.insert("counter");
    ignoredWords.insert("fast");
    ignoredWords.insert("quickly");
    ignoredWords.insert("rapidly");
    ignoredWords.insert("swiftly");
    ignoredWords.insert("slowly");
    ignoredWords.insert("sluggishly");
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
        commandList.push_back( CommandVector(commandType) );
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
        interpretPhraseElements( *temp, *tempCommand );
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
                //althought the second command might be valid, it is currently not supported
                if( currentCommand->loc != L_noSet ){
                    if( currentCommand->distance != FLOAT_NOSET || currentCommand->angle != FLOAT_NOSET ){
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
    //TODO: set defaults for commands that are not invalidated
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
                    }else{
                        //otherwise default distance
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
    //if an invalid iterator was given
    if( *root == NULL ) return potentialCommandWordList;
    if( (*root)->getType() != NLP::root ) return potentialCommandWordList;
    //recursive call
    depthCommandFind( root, potentialCommandWordList );
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

void Interpreter::depthCommandFind( NLP::LinguisticTree::Iterator& current, std::list<NLP::LinguisticTree::Iterator>& potentialCommands ){
    for( unsigned int i = 0; i < current.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = current.getChild(i);
        switch( getType(child) ){
            case NLP::verbBase:
            case NLP::verbPresent:
                potentialCommands.push_back( child );
                break;
            case NLP::sentence:
            case NLP::verbPhrase:
                depthCommandFind( child, potentialCommands );
                break;
            default:
                break;
        }
    }
    return;
}

void Interpreter::interpretPhraseElements( NLP::LinguisticTree::Iterator& word, CommandVector& command ){
    //grab the parent verb phrase
    NLP::LinguisticTree::Iterator directParent = word.getParent();
    //Go through all the phrases
    for( unsigned int i = 0; i < directParent.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator currentPhrase = directParent.getChild(i);
        if( currentPhrase == word ) continue;
        interpretPhrase( currentPhrase, command );
    }
}

void Interpreter::interpretPhrase( NLP::LinguisticTree::Iterator& phrase, CommandVector& command ){
    switch( getType(phrase) ){
        //interpret adverbs and particles
        case NLP::advPhrase:
        case NLP::phrasalVerbPart:
        //just in case of misclassification
        case NLP::adjPhrase:
        case NLP::verbPhrase:
            interpretAdverb( phrase, command );
            break;
        case NLP::prepPhrase:
            interpretPreposition( phrase, command );
            break;
        default:
            break;
    }
}

void Interpreter::interpretAdverb( NLP::LinguisticTree::Iterator& phrase, CommandVector& command ){
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
                try{
                    //if it is an augmentation adverb grab it's augment
                    currentAugment = &adverbAugmentMap.at( NLP::LinguisticTree::getText(child) );
                }catch( std::out_of_range oor ){
                    //otherwise check to see if we can ignore it
                    if( !isIgnored( child ) ){
                        //if we can't it's an erroneous command
                        if( statusState[error] ){
                            cerr<<"Invalidating adverb \""<<NLP::LinguisticTree::getText(child)<<"\". Invalidating Command."<<endl;
                        }
                        invalidate( command );
                        return;
                    }
                }
                break;
            //if nested needs to be evaluated:
            case NLP::advPhrase:
            case NLP::adjPhrase:
                interpretAdverb( child, command );
            default:
                //TODO: handle unrecognized types
                break;
        }
        //check to see if we caught an Augment
        if( currentAugment != NULL ){
            //check to make sure it executes:
            if( !currentAugment->augment( command, child ) ){
                //if it fails the command is erroneous:
                if( statusState[error] ){
                    cerr<<"Augmentation failed. Parameter redundancy detected."<<endl;
                }
                invalidate( command );
                return;
            }
        }
    }
}

void Interpreter::interpretPreposition( NLP::LinguisticTree::Iterator& phrase, CommandVector& command ){
    //first child is always the preposition:
    //TODO: Try to find a counter-example
    NLP::LinguisticTree::Iterator child = phrase.getFirstChild();
    const char* preposition = NLP::LinguisticTree::getText(child);
    //want to replace if else block with a map later:
    if( !strcmp( "to", preposition ) ){
        //if it is a to then we have a location
        if( command.baseType == FC_goLocation ){
            //if the command is a direct object location type like "find" or "approach"
            //the command is invalidated i.e. "find to the chair" doesn't make sense.
            if( statusState[error] ){
                cerr<<"invalid preposition \"to\" found in command of type GoTo"<<endl;
            }
            invalidate(command);
            return;
        }
        child = child.getNextSibling();
        interpretLocation( child, command );
    }else{
        invalidate( command );
        if( statusState[error] ){
            cerr<<"Unrecognized preposition \""<<preposition<<"\". Invalidating Command"<<endl;
        }
    }
}

void Interpreter::interpretLocation( NLP::LinguisticTree::Iterator& phrase, CommandVector& command ){
    if( getType(phrase) != NLP::nounPhrase ){
        invalidate( command );
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
            //possible misclassifications
            case NLP::adjective:
                try{
                    //if it is an augmentation adverb grab it's augment
                    currentAugment = &locAugmentMap.at( NLP::LinguisticTree::getText(child) );
                }catch( std::out_of_range oor ){
                    //otherwise check to see if we can ignore it
                    if( !isIgnored( child ) ){
                        //if we can't it's an erroneous command
                        if( statusState[error] ){
                            cerr<<"Invalidating location noun \""<<NLP::LinguisticTree::getText(child)<<"\". Invalidating Command."<<endl;
                        }
                        invalidate( command );
                        return;
                    }
                }
                break;
            //in case of nesting
            case NLP::nounPhrase:
            //or phrase misclassification
            case NLP::adjPhrase:
                interpretAdverb( child, command );
                break;
            default:
                if( !isIgnored( child ) ){
                    if( statusState[error] ){
                        cerr<<"Warning: Unsupported type "<<getType(child)<<" in location interpretation."<<endl;
                    }
                }
                break;
        }
        if( currentAugment != NULL ){
            //check to make sure it executes:
            if( !currentAugment->augment( command, child ) ){
                //if it fails the command is erroneous:
                if( statusState[error] ){
                    cerr<<"Augmentation failed. Parameter redundancy detected."<<endl;
                }
                invalidate( command );
                return;
            }
        }
    }

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
    if( dir != D_noChange ){
        //same as before
        if( command.dir != D_noSet ){
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
        case L_chair:
            return "Chair";
        case L_default:
            return "Default";
        default:
            break;
    }
    return "";
}
