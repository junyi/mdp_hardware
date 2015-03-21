#ifndef ModeFilter_h
#define ModeFilter_h

#define MOD_FILTER_SIZE 30

#include <inttypes.h>
#include <HashMap.h>

class ModeFilter
{
  private:
  public:
        ModeFilter();

        void             insert(int _sample);
        int16_t         _samples[MOD_FILTER_SIZE];
        int16_t         mode();

  private:
  		void 			quickSort(int arr[], int left, int right);
        void            isort();
        int8_t          _sample_index;
};

#endif