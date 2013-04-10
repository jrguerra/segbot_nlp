#!/usr/bin/env perl 
#===============================================================================
#
#         FILE: scrape.pl
#
#        USAGE: ./scrape.pl  
#
#  DESCRIPTION: 
#
#      OPTIONS: ---
# REQUIREMENTS: ---
#         BUGS: ---
#        NOTES: ---
#       AUTHOR: Anthony DuBois (), 
# ORGANIZATION: 
#      VERSION: 1.0
#      CREATED: 01/23/2013 06:36:37 AM
#     REVISION: ---
#===============================================================================

# Specify the URL of the page to post to.
my $URLtoPostTo = "http://nlp.stanford.edu:8080/corenlp/";

my $input = $ARGV[0];

if ($input =~ /^robot(.+)$/i || $input =~ /^carlton(.+)$/i || $input =~ /^please(.+)$/i) {
	$input = $1;
}
$input = "Go and $input";

$input =~ s/\*/times/g;
$input =~ s/\r/return/g;
$input =~ s/\t/tab/g;
$input =~ s/(\w)\n(\w)/$1 return $2/g;
$input =~ s/\n(\w)/return $1/g;
$input =~ s/(\w)\n/$1 return/g;
$input =~ s/\n/return/g;

##print "$input\n";

my %Fields = (
   "outputFormat" => "xml",
   "input" => "$input",
);

my $BrowserName = "";

use Digest::MD5  qw(md5_hex);
use LWP::UserAgent;
use HTTP::Request::Common;
use HTML::FormatText::WithLinks;

# Create the browser that will post the information.
my $Browser = new LWP::UserAgent;

# Insert the browser name, if specified.
if($BrowserName) { $Browser->agent($BrowserName); }

# Post the information to the CGI program.
my $Page = $Browser->request(POST $URLtoPostTo,\%Fields);

# Print the returned page (or an error message).
if ($Page->is_success) { 
	$output = $Page->content;
	my $f = HTML::FormatText::WithLinks->new();
	$output = $f->parse($output);
	$output =~ s/.*?(<.xml.*<\/root>).*/$1/sg;
	$output =~ s/^\s\s\s//mg;
	$output =~ /(<parse>.*<\/parse>)/s;
	$parseseg = $1;
	$parseseg =~ s/\n//sg;
	$output =~ s/<parse>.*<\/parse>/$parseseg/sg;
	$output = $output."\n";
	$name = "/home/nlpros/ros/rosbuild_ws/segbot/segbot_nlp/src/gspeech/".md5_hex($input).".xml";
	open $FILE, ">", $name;
	print $FILE "$output\n";
	close $FILE;
	print $name;
	
}
else { print $Page->message; }

#print "\n***************** Input to Stanford NLP *****************\n";
#print "$input\n";
#print "**********************************************************\n";
# end of script

