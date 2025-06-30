#ifndef RANGE_H
#define RANGE_H

class Range{
    public:
        struct Range_t
        {
            float range;
            float field_of_view;
            float min_range;
            float max_range;
        };
        void initRange();
        Range_t getRange();
    private:
        Range_t range_msg_;
};

#endif
