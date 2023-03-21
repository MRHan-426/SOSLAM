/* ----------------------------------------------------------------------------

定义了一个异常类QuadricProjectionException。
这个异常类继承自gtsam::ThreadsafeException<QuadricProjectionException>，
是一个线程安全的异常类，用于在计算四面体投影时抛出异常。在该类中，定义了三个公有成员函数：

1.构造函数：可以有无参构造函数，可以有一个参数的构造函数，也可以有一个字符串类型的构造函数。

2.nearbyVariable()：返回一个gtsam::Key类型的变量，表示出问题的变量。

3.私有成员变量：一个gtsam::Key类型的变量j_，表示出问题的变量。

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricProjectionException.h
 * @date May 19, 2020
 * @author Lachlan Nicholson
 * @brief An exception to be thrown when projecting a quadric has failed
 */

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/inference/Key.h>

#include <string>

namespace gtsam_quadrics
{

    /**
     * @class QuadricProjectionException
     * Exception thrown when attemption to calculate quadric bounding box fails
     */
    class QuadricProjectionException
        : public gtsam::ThreadsafeException<QuadricProjectionException>
    {
    public:
        QuadricProjectionException()
            : QuadricProjectionException(std::numeric_limits<gtsam::Key>::max()) {}

        QuadricProjectionException(gtsam::Key j)
            : ThreadsafeException<QuadricProjectionException>(
                  "QuadricProjectionException"),
              j_(j) {}

        QuadricProjectionException(const std::string &description)
            : ThreadsafeException<QuadricProjectionException>(description) {}

        gtsam::Key nearbyVariable() const { return j_; }

    private:
        gtsam::Key j_;
    };

} // namespace gtsam_quadrics
