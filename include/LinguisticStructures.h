/**	@name LinguisticStructures.h
  *
  *	Contains Range and Dependency classes
  *
  *	@author		C. Daniel LaBar
  *	Date Created:
  *	@date		2013-03-29
  *	Version Info:
  *	@version	1.00.00
  *	@date		2013-03-30
  *
  *	Version History:
  *	@version	0.01.00
  *	@date		2013-03-29
  *	TODO: dependency maps
  *	@version	1.00.00
  *	@date		2013-03-30
  * @author Joe Martinez
  *	NLP::Dependency::type has been completed.
  *
**/
#ifndef LINGUISTICSTRUCTURES_H
#define LINGUISTICSTRUCTURES_H

#define LINGUISTIC_RANGE_INVALID -1

namespace NLP{
    class Range{
        public:
            Range();
            Range( int value );
            Range( int startIndex, int endIndex );

        private:
            int startIndex, endIndex;

        public:
            int getFirst();

            bool contains( int index );

            bool setRange( int begin, int end );

            bool isValid();
    };

    class Dependency{
        public:
            enum type{ abbrev,acomp, advel, advmod, agent, amod, appos, attr, aux, auxpass,
                        cc, ccomp, complm, conj, cop, csubj, csubjpass, dep, det, dobj,
                        expl, infmod, iobj, mark, mwe, neg, nn, npadvmod, nsubj, nsubjpass,
                        num, number, parataxis, partmod, pcomp, pobj, poss, possessive, preconj, predet,
                        prep, prepc, prt, punct, purpcl, quantmod, remod, refer, rel, root,
                        tmod, xcomp, xsubj, numTypes, unrecognizedType };

            Dependency( Dependency::type identification, int governor, int dependent );

        private:
            Dependency::type identification;
            int governor;
            int dependent;

        public:
            Dependency::type getType();
            int getGovernor();
            int getDependent();

    };

    class CoreferenceIdentifier{
        public:
            CoreferenceIdentifier();
            CoreferenceIdentifier( int coref, int index );

        private:
            int coref;
            int corefIndex;

        public:
            int getCoref();
            int getIndex();

    };
}

#endif // LINGUISTICSTRUCTURES_H
