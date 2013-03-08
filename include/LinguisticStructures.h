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
            enum type{ first, unrecognizedType };

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
