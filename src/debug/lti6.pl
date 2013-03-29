#!usr/bin/perl
use warnings;
use strict;

my $name = $ARGV[0];

#Global Variables
my %tokens = ();
my %coms = ();
my @comWords = ();

my $sentence = "";

my @commandTokens = ();
my @movementCommands = ("move", "go", "turn", "stop", "drive", "travel");
my @danceCommands = ("dance", "boogie");
my @otherCommands = ("halt");
my @specialWords = ("left");

my @preps = ("to");
my @times = ("twice", "thrice");
my @numerical = ("meter", "degree", "time");
my @dirs = ("forward", "backward", "left", "right", "around", "ahead", "counterclockwise", "clockwise");
my @history = ("again");
push(@commandTokens, @movementCommands, @danceCommands, @otherCommands);

my @dests = ("wall", "bookcase", "hallway");
my @dest_dirs = ("left", "right");

my @objects = ();
push (@objects, @numerical, @dirs, @history, @dests);

my @limiters = ("and", "but", "then");
push (@limiters, @commandTokens); ## these will limit command exploration in a sentence

## isMem(array, val)
sub isMem {
	## argument 1 is the array, arg2 is the val
	my @array = @{$_[0]};
	my $val = $_[1];
	
	##print ".... isMem($val)\n";
	foreach (@array) {
		##print "...... checking $_ against $val\n";
		if ($_ eq $val) {
			return 1;
		}
	}
	
	return 0;
}

## initialization
open(STDIN, $name);

print "Step 1. Processing input file and assembling command words\n";
my $inputState = -1; # 0 is for tokens
while (<STDIN>) {
	my $line = $_;
	
	if ($line =~ /tokens/) {
		print ".. Collecting basic tokens\n";
		$inputState = 0;
	} elsif ($line =~ /basic-dependencies/) {
		print ".. Collecting basic dependencies\n";
		$inputState = 1;
	} elsif ($line =~ /coreference/i && $inputState != 2) {
		print ".. Collecting coreference resolution\n";
		$inputState = 2;
	}
	
	
	if ($inputState == 0) {
		token($line);
	}
	
	if ($inputState == 1) {
		dependency_map($line);
	}
	
	if ($inputState == 2) {
		coref_map($line);
	}
}
close (STDIN);

print "\nCommand Tokens Found:\n";
foreach (@comWords) {
	print " -- $tokens{$_}{lemma}\n";
}

print "\nStep 2. Assessing Recognized Commands for Commandability\n";
my @Commands = ();
while ($#comWords >= 0) {
	my $word = pop(@comWords);
	print ".. Assessing Command $tokens{$word}{lemma}\n";
	
	if (ProcessPartOfSpeech($word) != 1) {
		next;
	}
	
	if (isMem(\@movementCommands, $tokens{$word}{lemma}) == 1) {
		print ".... $tokens{$word}{lemma} was identified as a Movement\n";
		my $command = AssessMovementCommand($word);
		push (@Commands, $command);
	} elsif (isMem(\@danceCommands, $tokens{$word}{lemma}) == 1) {
		print ".... $tokens{$word}{lemma} was identified as a Dance\n";
		push (@Commands, "$word DANCE(ABC) LOC:X DIR:X ANGLE:X DIST:X NUM:X HIST:X"); ## this will need to be changed for assessmbling other things
	} else {
		print ".... $tokens{$word}{lemma} was identified as Other\n";
		##AssessHaltCommand($word);
	}
}

sub ProcessPartOfSpeech {
	my $word = shift;
	
	print "... $tokens{$word}{lemma} is $tokens{$word}{pos}\n";
	if ($tokens{$word}{pos} =~ /vb/i) {
		print "... $tokens{$word}{lemma} is a verb\n";
		my $subj = -1;
		
		my @acceptableSubjects = ("carlton","Carlton", "you", "robot", "segway", "segbot");
		
		if ($tokens{$word}{lemma} ne $tokens{$word}{word}) {
			print "... $tokens{$word}{lemma} is in incorrect tense : $tokens{$word}{word}\n";
			return 0;
		}
		
		my @deps = keys(%{$tokens{$word}{deps}});
		foreach my $dep (@deps) {
			my $type = $tokens{$word}{deps}{$dep};
			if ($type =~ /subj/i) {
				if (isMem(\@acceptableSubjects, $tokens{$dep}{lemma}) == 0) {
					if (isMem(\@objects, $tokens{$dep}{lemma}) == 1) {
						my $validCommandExistsBeforeObject = 0;
						for (my $i = $dep - 1; $i > 0; $i --) {
							if (isMem(\@commandTokens, $tokens{$i}{lemma}) == 1) {
								if (isMem(\@movementCommands, $tokens{$i}{lemma}) == 1) {
									$validCommandExistsBeforeObject = 1;
									last;
								} else {
									$validCommandExistsBeforeObject = 0;
									last;
								}	
							}
						}

						if ($validCommandExistsBeforeObject == 1) {
							print ".... $tokens{$word}{lemma} is connected to proper object: $tokens{$dep}{lemma}\n";
							return 1; ## this is a guess on my part that this will work
						} else {
							next; ## keep on looking for that subject
						}

					}

					print "... $tokens{$word}{lemma} has an unacceptable subject: $tokens{$dep}{lemma}\n";
					return 0;
				} else {
					print "... $tokens{$word}{lemma} has acceptable subject: $tokens{$dep}{lemma}\n";
					$subj = $dep;
					last;
				}
			}
		}
		
		my @subjectRequiringHelpers = ("should", "want");
		
		foreach my $dep (@deps) {
			my $type = $tokens{$word}{deps}{$dep};
			if (($type =~ /aux/i) || ($type =~ /xcomp/i)) {
				if (isMem(\@subjectRequiringHelpers, $tokens{$dep}{lemma}) == 1) {
					if ($subj == -1) {
						print "... $tokens{$word}{lemma} has helper word '$tokens{$dep}{lemma}' but no viable subject\n";
						return 0; ## this is an error, it needs a subject
					}
				}
			}
		}
		
		print "... Checking objects\n";
		if (isMem(\@movementCommands, $tokens{$word}{lemma}) == 1) {
			foreach my $dep (@deps) {
				if ($tokens{$word}{deps}{$dep} =~ /obj/i) {
					print "... $tokens{$word}{lemma} is directly object connected to $tokens{$dep}{lemma}\n";
					my $nPt = NounPhraseSearch($dep, $word);
					my @nouns = @{$nPt};
					foreach my $noun (@nouns) {
						print "... $tokens{$word}{lemma} is indirectly object connected to $tokens{$noun}{lemma}\n";
						if (isMem(\@dests, $tokens{$noun}{lemma}) == 1) {
							print "... $tokens{$word}{lemma} has an invalid direct object: $tokens{$noun}{lemma}\n";
							return 0;
						}
					}
				}
			}
		}
	}
	
	if ($tokens{$word}{pos} =~ /nn/i) {
		print ".... $tokens{$word}{lemma} is a Noun\n";
		
		if ($tokens{$word}{pos} =~ /nns/i) {
			print ".... $tokens{$word}{lemma} is invalid pos: $tokens{$word}{pos}\n";
			return 0;
		}
		
		my @deps = keys(%{$tokens{$word}{deps}});
		foreach my $dep (@deps) {
			if (isMem(\@commandTokens, $tokens{$dep}{lemma}) == 1) {
				if ($tokens{$dep}{pos} =~ /vb/i) {
					my $result = ProcessPartOfSpeech($dep);
					if ($result) { 
						print ".... $tokens{$word}{lemma} is connected to proper command: $tokens{$dep}{lemma}\n";
						return 1;
					}
				}
			}
		}
		
		foreach my $dep (@deps) {
			if (isMem(\@objects, $tokens{$dep}{lemma}) == 1) {
				my $validCommandExistsBeforeObject = 0;
				for (my $i = $dep - 1; $i > 0; $i --) {
					if (isMem(\@commandTokens, $tokens{$i}{lemma}) == 1) {
						if (isMem(\@movementCommands, $tokens{$i}{lemma}) == 1) {
							$validCommandExistsBeforeObject = 1;
							last;
						} else {
							$validCommandExistsBeforeObject = 0;
							last;
						}	
					}
				}
				
				if ($validCommandExistsBeforeObject == 1) {
					print ".... $tokens{$word}{lemma} is connected to proper object: $tokens{$dep}{lemma}\n";
					return 1; ## this is a guess on my part that this will work
				}

			}
		}
		
		foreach my $dep (@deps) {
			if ($tokens{$dep}{pos} =~ /dt/i) {
				return 0;
			}
		}
	}
	
	return 1;
}

print "\n\n****************************************\n";
print "*************** Commands ***************\n";
print "****************************************\n";

print "Input: $sentence";
print "\n\n";

my $i = 1;
my @fcs = ();
foreach my $c (reverse(@Commands)) {
	my $x = FormatCommand($c);
	if ($x ne "") {
		$fcs[$#fcs + 1] = $x;	
	}
	
}

open (FILE, "> $name.txt");
foreach (@fcs) {
	print FILE "$_\n";
}
close (FILE);

print "\n\n";

my $tokenId = 1;
sub token {
	my $string = $_[0];
	
	if ($string =~ /token id=\"(\d+)\"/) {
		$tokenId = $1;
	}
	
	if ($string =~ /<word>(\w+)<\/word>/) {
		$tokens{$tokenId}{word} = $1;
		print ".... Token $tokenId:\$tokenHash{$tokenId}{word} = $1\n";
		$sentence = "$sentence $1";
	}
	
	if ($string =~ /<lemma>(\w+)<\/lemma>/) {
		if (isMem(\@specialWords, $tokens{$tokenId}{word})) {
			$tokens{$tokenId}{lemma} = $tokens{$tokenId}{word};
		} else {
			$tokens{$tokenId}{lemma} = $1;	
		}
		print ".... Token $tokenId:\$tokenHash{$tokenId}{lemma} = $1\n";
		if (isMem(\@commandTokens, $tokens{$tokenId}{lemma}) == 1) {
			push(@comWords, $tokenId);
		}
		
	}
	
	if ($string =~/<POS>(\w+)<\/POS>/) {
		$tokens{$tokenId}{pos} = $1;
		print ".... Token $tokenId:\$tokenHash{$tokenId}{pos} = $1\n";
		
	}
}

my $currentDep = "";
my $gov;
my $dep;
sub dependency_map {
	my $string = $_[0];
	
	if ($string =~ /dep type=\"(\w+)\"/) {
		$currentDep = $1;
	}
	
	if ($string =~ /governor idx=\"(\d+)\"/) {
		$gov = $1;
	}
	
	if ($string =~ /dependent idx=\"(\d+)\"/) {
		$dep = $1;
	}
	
	if ($string =~ /<\/dep>/) {
		$tokens{$gov}{deps}{$dep} = $currentDep;
		$tokens{$dep}{deps}{$gov} = $currentDep;
		print ".... $tokens{$gov}{lemma} <= $currentDep => $tokens{$dep}{lemma}\n";
	}
}

my $coref_open = 0;
my %first = ();
my %second = ();
sub coref_map {
	my $line = shift;
	
	if ($line =~ /<coreference>/i) {
		$coref_open = 1;
	}
	
	if ($line =~ /<\/coreference>/) {
		$coref_open = 0;
		%first = ();
		%second = ();
	}
	
	if ($coref_open) {
		if ($line =~ /<start>(\d+)<\/start>/) {
			if (! exists($first{start})) {
				$first{start} = $1;
				print ".... coref_map: Setting \$first{start} = $1\n";
			} else {
				$second{start} = $1;
				print ".... coref_map: Setting \$second{start} = $1\n";
			}
		}
		
		if ($line =~ /<end>(\d+)<\/end>/) {
			if (!exists($first{end})) {
				$first{end} = $1;
				print ".... coref_map: Setting \$first{end} = $1\n";
			} else {
				$second{end} = $1;
				print ".... coref_map: Setting \$second{end} = $1\n";
				
				my $m1 = -1;
				my $m2 = -1;
				for (my $i = $first{start}; $i <= $first{end}; $i ++) {
					print ".... coref_map: .. Checking $tokens{$i}{lemma}\n";
					if ($tokens{$i}{pos} =~ /prp/i) {
						$m1 = $i;
						last;
					}
					
					if (isMem(\@dests, $tokens{$i}{lemma}) == 1) {
						$m1 = $i;
						last;
					}
				}
				
				for (my $i = $second{start}; $i <= $second{end}; $i ++) {
					print ".... coref_map: .. Checking $tokens{$i}{lemma}\n";
					if ($tokens{$i}{pos} =~ /prp/i) {
						$m2 = $i;
						last;
					}
					
					if (isMem(\@dests, $tokens{$i}{lemma}) == 1) {
						$m2 = $i;
						last;
					}
				}
				
				if (($m1 == -1) || ($m2 == -1)) {
					print ".. coref_map: m1 $m1 m2 $m2\n";
					return;
				}
				
				print ".... coref_map: Found $tokens{$m1}{lemma} and $tokens{$m2}{lemma}\n";
				
				if ($tokens{$m1}{pos} =~ /prp/i && isMem(\@dests, $tokens{$m2}{lemma})) {
					print ".... coref_map: Setting $tokens{$m1}{lemma} to $tokens{$m2}{lemma}\n";
					$tokens{$m1}{lemma} = $tokens{$m2}{lemma};
					$tokens{$m1}{pos} = $tokens{$m2}{pos};
				} elsif ($tokens{$m2}{pos} =~ /prp/i && isMem(\@dests, $tokens{$m1}{lemma})) {
					print ".... coref_map: Setting $tokens{$m2}{lemma} to $tokens{$m1}{lemma}\n";
					$tokens{$m2}{lemma} = $tokens{$m1}{lemma};
					$tokens{$m2}{pos} = $tokens{$m1}{pos};
				}		
			}
		}
	}
}

my %finalCommand = ();
sub AssessMovementCommand {
	my $word = $_[0]; ## token id for this command token
	my %retVal = ();
	
	%finalCommand = ();
	
	print ".... Checking dependencies\n";
	CollectDependencies($word);
	
	if (! exists($finalCommand{location})) {
		$finalCommand{location} = "default";
	}

	if (! exists($finalCommand{angle})) {
		$finalCommand{angle} = "default";
	}
	
	if (! exists($finalCommand{direction})) {
		$finalCommand{direction} = "default";
	}
	
	if (! exists($finalCommand{distance})) {
		$finalCommand{distance} = "default";
	}
	
	if (! exists($finalCommand{numTimes})) {
		$finalCommand{numTimes} = "default";
	}
	
	if (! exists($finalCommand{history})) {
		$finalCommand{history} = "default";
	}
	
	my $retVal = "$word MOVE($tokens{$word}{lemma}) LOC:$finalCommand{location} DIR:$finalCommand{direction} ANGLE:$finalCommand{angle} DIST:$finalCommand{distance} NUM:$finalCommand{numTimes} HIST:$finalCommand{history}";
	return $retVal;	
	
}

sub CollectDependencies {
	my $comm = $_[0];
	
	print "........ CollectDep($comm)\n";

	my %visited = ();
	$visited{$comm} = 1; ## don't want to go back the command
	
	my @mapStack = ();
	push (@mapStack, keys(%{$tokens{$comm}{deps}})); ## push the first level dependencies
	
	while ($#mapStack >= 0) {
		my $dep = pop(@mapStack);
		
		if (exists($visited{$dep})) {
			next;
		}
		
		if ($dep < $comm) {
			next;
		}
		
		print "........ Checking Dependency $tokens{$dep}{lemma}: \n";
		
		$visited{$dep} = 1;
		
		if (!exists($tokens{$dep}{used})) {
			my $used = 0;
			if ((isMem(\@numerical, $tokens{$dep}{lemma}) == 1)) {
				my $num = $dep - 1; ## this is a thing!
				my $foundFlag = 1; ## basically we take a guess, but I think it's safe

				if ($foundFlag == 1) {
					if ($tokens{$dep}{lemma} =~ /meter/ && !exists($finalCommand{distance})) {
						$finalCommand{distance} = $tokens{$num}{lemma};
						print "........ Setting Distance to $num\n";
						$used = 1;
					} elsif ($tokens{$dep}{lemma} =~ /degree/ && !exists($finalCommand{angle})) {
						print "........ Setting angle to $num\n";
						$finalCommand{angle} = $tokens{$num}{lemma};
						$used = 1;
					} elsif ($tokens{$dep}{lemma} =~ /time/ && !exists($finalCommand{numTimes})) {
						print "........ Setting numTimes to $num\n";
						$finalCommand{numTimes} = $tokens{$num}{lemma};
						$used = 1;
					}
				}
			}

			if (isMem(\@times, $tokens{$dep}{lemma}) == 1 && !exists($finalCommand{numTimes})) {
				AssessNumTimes($dep, $comm);
			}

			if ((isMem(\@preps, $tokens{$dep}{lemma}) == 1) && !exists($finalCommand{location})) {
					print "...... Found '$tokens{$dep}{lemma}'\n";
					AssessPreposition($dep, $comm);
			}

			if ((isMem(\@dirs, $tokens{$dep}{lemma}) == 1) && (!exists($finalCommand{direction}))) {
				$finalCommand{direction} = $tokens{$dep}{lemma};
				print "........ Setting direction to $tokens{$dep}{lemma}\n";
				$used = 1;
			}

			if ((isMem(\@history, $tokens{$dep}{lemma}) == 1) && !exists($finalCommand{history})) {
				$finalCommand{history} = 1;
				$used = 1;
			}
			
			if ($used == 1) {
				$tokens{$dep}{used} = 1;
			}
		} else {
			print "........ Cannot Use $tokens{$dep}{lemma}. Already in Use.\n"
		}
		
		my @nextCommands = reverse(sort(keys(%{$tokens{$dep}{deps}})));
		foreach my $next (@nextCommands) {
			if ($tokens{$dep}{deps}{$next} =~ /cc/) {
				next;
			} elsif ($tokens{$dep}{deps}{$next} =~ /conj/) {
				next;
			} 	elsif ($tokens{$dep}{deps}{$next} =~ /conj/) {
					next;
			} elsif (exists($visited{$next})) {
				next;
			} elsif ($next < $comm) {
				next;
			} else {
				$mapStack[$#mapStack + 1] = $next; 
			}	
		}
		
	}	
}

sub AssessPreposition {
	my $prep = $_[0];
	my $comm = $_[1];
	
	print "........ AssessPrep($prep, $comm)\n";
	
	my %visited = ();
	$visited{$comm} = 1; ## don't want to go back the word itself!
	$visited{$prep} = 1;
	
	my @mapStack = reverse(sort(keys(%{$tokens{$prep}{deps}})));
	while ($#mapStack >= 0) {
		my $dep = pop(@mapStack);
		
		if (exists($visited{$dep})) {
			next;
		}
		
		print "........ Checking Dependency $tokens{$dep}{lemma}: \n";
		
		$visited{$dep} = 1;
		
		if (! exists($tokens{$dep}{used})) {
			if (isMem(\@dests, $tokens{$dep}{lemma}) == 1) {
				print "........ '$tokens{$prep}{lemma}' maps to a destination '$tokens{$dep}{lemma}'\n";
				$finalCommand{location} = $tokens{$dep}{lemma};
				$tokens{$dep}{used} = 1;
				return;
			}

			if (isMem(\@dest_dirs, $tokens{$dep}{lemma}) == 1) {
				print "........ '$tokens{$prep}{lemma}' maps to a direction '$tokens{$dep}{lemma}'\n";
				$finalCommand{direction} = $tokens{$dep}{lemma};
				$tokens{$dep}{used} = 1;
				return;
			}	
		}

		my @nextCommands = reverse(sort(keys(%{$tokens{$dep}{deps}})));
		foreach my $next (@nextCommands) {
			if ($tokens{$dep}{deps}{$next} =~ /cc/) {
				next;
			} elsif ($tokens{$dep}{deps}{$next} =~ /conj/) {
				next;
			} elsif (exists($visited{$next})) {
				next;
			} elsif ($next < $comm) {
				next;
			} else {
				$mapStack[$#mapStack + 1] = $next; 
			}	
		}	
	}	
}

sub NounPhraseSearch {
	my $word = shift;
	my $comm = shift;
	my @retVal = (); ## return an array of nouns!
	
	print "...... NPS: $tokens{$word}{lemma}, $tokens{$comm}{lemma}\n";
	
	my %visited = ();
	$visited{$comm} = 1;
	
	my @mapStack = ($word);
	while ($#mapStack >= 0) {
		my $dep = pop(@mapStack);
		print "...... NPS: ** Current Noun in Search is $tokens{$dep}{lemma}\n";
		
		if (exists($visited{$dep})) {
			next;
		}
		
		print "........ Checking Dependency $tokens{$dep}{lemma}: \n";
		
		$visited{$dep} = 1;
		
		my $acceptableNoun = 1;
		my @modifiers = keys(%{$tokens{$dep}{deps}});
		foreach my $mod (@modifiers) {
			if ($tokens{$dep}{deps}{$mod} =~ /amod/i) {
				## we do not support adjectives, they are just too hard
				$acceptableNoun = 0;
				last;
			}
		}
		
		if ($acceptableNoun == 1) {
			print "...... NPS: **** Adding $tokens{$dep}{lemma} to retVal\n";
			push(@retVal, $dep); ## we can add it
		}
		
		foreach my $mod (@modifiers) {
			print "...... NPS: **** Choosing to Explore $tokens{$mod}{lemma}\n";
			if (($tokens{$mod}{pos} =~ /nn/i) && !(exists($visited{$mod}))) {
				if ($dep > $comm) {
					push (@mapStack, $mod);	
				}
			}
		}	
	}
	
	return \@retVal; ## returns all the collected nouns
}

sub AssessNumTimes {
	my $word = $_[0];
	my $comm = $_[1];
	
	if ($tokens{$dep}{lemma} =~ /twice/) {
		$finalCommand{numTimes} = "2";
		$tokens{$word}{used} = 1;
		print "........ Num Times set to 2\n";
		return;
	}
	
	if ($tokens{$dep}{lemma} =~ /thrice/) {
		$finalCommand{numTimes} = "3";
		$tokens{$word}{used} = 1;
		print "........ Num Times set to 2\n";
		return;
	}

}

my @FormattedCommands = ();
sub FormatCommand {
	my $c = shift;
	print "$c\n";
	if ($c =~ /(\w+) (\w+)\((\w+)\) LOC:(\w+) DIR:(\w+) ANGLE:(\w+) DIST:(\w+) NUM:(\w+) HIST:(\w+)/) {
		my $index = $1;
		my $type = $2;
		my $cword = $3;
		my $loc = $4;
		my $dir = $5;
		my $angle = $6;
		my $dist = $7;
		my $numt = $8;
		my $hist = $9;
		
		print "** Reducing Command..\n";
		
		if ($hist =~ /default/i) {
			print "*** History set to 0\n";
			$hist = 0;
		} else {
			print "*** History set to 1\n";
			$hist = 1;
		}
		
		if ($numt =~ /default/i) {
			print "*** NumTimes set to 0\n";
			$numt = 0;
		} else {
			print "*** NumTimes set to $numt\n";
		}
		
		if ($type =~ /move/i) {
			if ($cword =~ /turn/) {
				## this is an exception
				print "*** turn type command...\n";
				if ($loc !~ /default/i) { 
					print "**** Location was set! Error!\n";
					return;
				}
				if ($dist !~ /default/i) { 
					print "**** Distance was set! Error!\n";
					return;
				}
				if ($dir =~ /forward/i || $dir =~ /ahead/i) { 
					print "**** Direction was set to $dir! Error!\n";
					return;
				}
				if ($dir =~ /backward/i && $angle !~ /default/i) { 
					print "**** Direction was set to Backward, but there was an angle set to $angle\n";
					return;
				} elsif ($dir =~ /backward/i && $angle =~ /default/i) {
					$angle = 180;
				}
				if ($dir =~ /around/i && $angle !~ /default/i) { 
					print "**** Direction was set to around, but there was an angle set to $angle\n";
					return;
				} elsif ($dir =~ /around/i && $angle =~ /default/i) {
					$angle = 358; ## make it turn around
				}
				
				if ($angle =~ /default/i) {
					$angle = 90;
				}
				
				if ($dir =~ /right/i || $dir =~ /clockwise/i) {
					$angle = -$angle;
				}
				
				$loc = 0;
				$dist = 0;
			} else {
				print "*** move word $cword\n";
				##my @dirs = ("forward", "backward", "left", "right", "around", "ahead", "counterclockwise", "clockwise");
				if ($dir =~ /backward/i && $angle !~ /default/i) { 
					return;
				} elsif ($dir =~/backward/i && $angle =~ /default/i) {
					$angle = 180;
				}
				
				if ($loc !~ /default/i) {
					if ($dist !~ /default/i || $dir !~ /default/i || $angle !~ /default/i) {
						return;
					}
					
					if ($cword =~ /stop/i) {
						return;
					}
					
					$dist = 0; $dir = 0; $angle = 0;
				} else {
					$loc = 0;
					print "**** Distance is set to default\n";
					if ($cword =~ /stop/i) {
						if ($angle !~ /default/i) {
							print "***** Angle is not default\n";
							return;
						}
						
						if ($dir !~ /ahead/i && $dir !~ /default/i) {
							print "***** Direction is not default\n";
							return;
						}
					}
					
					if ($dist =~ /default/i) {
						print "**** Distance is default\n";
						if ($cword =~ /stop/i) {
							$dist = 0;
						} else {
							$dist = 1;	
						}
						
						if ($angle =~ /default/i) {
							$angle = 0;
						}
						
						if ($dir =~ /right/i || $dir =~ /clockwise/i) {
							$angle = -$angle;
						}
						
					} elsif ($dist !~ /default/i) {
						if ($angle =~ /default/i) {
							$angle = 0;
						}
						
						if ($dir =~ /right/i || $dir =~ /clockwise/i) {
							$angle = -$angle;
						}
						
					} else {
						return;
					}
				}
				
			}
		} elsif ($type =~ /dance/i) {
			$loc = -1; $dir = 0; $angle = 0;
		} elsif ($type =~ /halt/i) {
			return;
		}	
	
		my $retVal = "$loc $dist $angle $numt $hist";
		##print "$retVal\n";
		return $retVal;
	}	
}
