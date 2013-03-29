#include "Interpreter.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
using namespace std;

//default distance in meters
#define DISTANCE_DEFAULT 5
//default angle of turn in degrees
#define ANGLE_DEFAULT 90

//all of our wonderfully annoying statics
std::map<std::string, fullCommand> Interpreter::commandMap;
static std::map<std::string, fullCommand> gerundReductionMap;
std::map<std::string, Interpreter::CommandAugment> Interpreter::adverbAugmentMap;
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
            if( nextWord == nextWord.getEnd() || strcmp( Interpreter::getText(nextWord), "now" ) ){
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

//function controls
CommandVector::CommandVector( fullCommand baseType ):baseType(baseType),trueType(Interpreter::reduce(baseType)),
    distance(FLOAT_NOSET),angle(FLOAT_NOSET),dir(D_noSet),loc(L_noSet),repetitions(REP_NOSET){}

void Interpreter::init(){
    if( isInit() ) return;
    //command map initialization
    commandMap["turn"] = FC_turn;
    commandMap["go"] = FC_move;
    commandMap["move"] = FC_move;
    commandMap["dance"] = FC_dance;

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
    //ignored terms
    ignoredWords.insert("now");
}

bool Interpreter::isInit(){
    return statusState.size();
}

void Interpreter::setStatus( Interpreter::status element, bool value ){
    if( !isInit() ) return;
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

    //TEST: for testing purposes only remove later
    for( std::list<NLP::LinguisticTree::Iterator>::iterator iter = commandWordList.begin(); iter != commandWordList.end(); ++iter ){
        cout<<(*(*iter))->getType()<<"  "<<iter->getNumChildren()<<"  "<<iter->getParentIndex()<<endl;
        cout<<((NLP::Word*) (*(*iter)) )->getText()<<endl;
    }

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
                    if( currentCommand->dir != D_left || currentCommand->dir != D_right ){
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
                //if this does not have a location
                if( command->loc == L_noSet ){
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
        NLP::type childType = (*child)->getType();
        switch(childType){
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
    switch( (*phrase)->getType() ){
        case NLP::advPhrase:
            interpretAdverb( phrase, command );
        default:
            break;
    }
}

void Interpreter::interpretAdverb( NLP::LinguisticTree::Iterator& phrase, CommandVector& command ){
    for( unsigned int i = 0; i < phrase.getNumChildren(); ++i ){
        NLP::LinguisticTree::Iterator child = phrase.getChild(i);
        CommandAugment* currentAugment = NULL;
        switch( (*child)->getType() ){
            case NLP::adverb:
            //just in case of bad classification
            case NLP::adjective:
                try{
                    //if it is an augmentation adverb grab it's augment
                    currentAugment = &adverbAugmentMap.at( getText(child) );
                }catch( std::out_of_range oor ){
                    //otherwise check to see if we can ignore it
                    if( !isIgnored( child ) ){
                        //if we can't it's an erroneous command
                        if( statusState[error] ){
                            cerr<<"Invalidating adverb \""<<getText(child)<<"\". Invalidating Command."<<endl;
                        }
                        invalidate( command );
                        return;
                    }
                }
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

const char* Interpreter::getText( NLP::LinguisticTree::Iterator& wordIter ){
    return ((NLP::Word*) *wordIter)->getText();
}

bool Interpreter::isIgnored( NLP::LinguisticTree::Iterator& wordIter ){
    return ignoredWords.find( getText(wordIter) ) != ignoredWords.end();
}

void Interpreter::invalidate( CommandVector& command ){
    command.baseType = FC_error;
    command.trueType = RC_error;
}

Interpreter::CommandAugment::CommandAugment( SpecialCase* special ):distance(FLOAT_NOCHANGE),angle(FLOAT_NOCHANGE),
    dir( D_noChange ), loc( L_noChange ),repetitions(REP_NOCHANGE),special(special){}

Interpreter::CommandAugment::CommandAugment( direction dir, location loc, float dis, float angle, int reps, SpecialCase* special )
    :distance(dis),angle(angle),dir(dir),loc(loc),repetitions(reps),special(special){}

bool Interpreter::CommandAugment::augment( CommandVector& command, NLP::LinguisticTree::Iterator& wordIter ){
    if( special != NULL ){
        if( !special->execute( command, wordIter ) ){
            return false;
        }
    }
    //if the distance is changed
    if( distance != FLOAT_NOCHANGE ){
        //if distance has already been set return failure i.e."Move 5 meters 4 meters"
        if( command.distance != FLOAT_NOSET ){
            return false;
        }
        //otherwise set the attribute
        command.distance = distance;
    }
    //if the angle is changed
    if( angle != FLOAT_NOCHANGE ){
        //same as before
        if( command.angle != FLOAT_NOSET ){
            return false;
        }
        command.angle = angle;
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
