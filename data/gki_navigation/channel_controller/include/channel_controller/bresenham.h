#ifndef BRESENHAM_H
#define BRESENHAM_H

namespace channel_controller
{

/// Bresenham line tracing
/**
 * Standard usage example:
 * for(Bresenham b(0, 0, 10, 5); b.hasNext(); b.advance()) {
 *     printf("%d %d\n", b.cur_x(), b.cur_y());
 * }
 */
class Bresenham
{
    public:
        Bresenham(int x0, int y0, int x1, int y1, unsigned int max_length = UINT_MAX);

        inline static int sign(int x)
        {
            return x > 0 ? 1.0 : -1.0;
        }

        bool hasNext() const;

        void advance();

        int cur_x() const { return cur_x_; }
        int cur_y() const { return cur_y_; }

    private:
        unsigned int abs_da;
        unsigned int abs_db;
        int error_b;
        int dx;
        int dy;
        bool go_x;
        int cur_x_;
        int cur_y_;
        unsigned int end;
        unsigned int i;
};

}

#endif

