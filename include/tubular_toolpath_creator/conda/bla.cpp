// set from a rotation matrix
template<typename Other>
struct quaternionbase_assign_impl<Other,3,3>
{
  typedef typename Other::Scalar Scalar;
  template<class Derived> EIGEN_DEVICE_FUNC static inline void run(QuaternionBase<Derived>& q, const Other& a_mat)
  {
    const typename internal::nested_eval<Other,2>::type mat(a_mat);
    EIGEN_USING_STD_MATH(sqrt)
    // This algorithm comes from  "Quaternion Calculus and Fast Animation",
    // Ken Shoemake, 1987 SIGGRAPH course notes
    Scalar t = mat.trace();
    if (t > Scalar(0))
    {
      t = sqrt(t + Scalar(1.0));
      q.w() = Scalar(0.5)*t;
      t = Scalar(0.5)/t;
      q.x() = (mat.coeff(2,1) - mat.coeff(1,2)) * t;
      q.y() = (mat.coeff(0,2) - mat.coeff(2,0)) * t;
      q.z() = (mat.coeff(1,0) - mat.coeff(0,1)) * t;
    }
    else
    {
      Index i = 0;
      if (mat.coeff(1,1) > mat.coeff(0,0))
        i = 1;
      if (mat.coeff(2,2) > mat.coeff(i,i))
        i = 2;
      Index j = (i+1)%3;
      Index k = (j+1)%3;

      t = sqrt(mat.coeff(i,i)-mat.coeff(j,j)-mat.coeff(k,k) + Scalar(1.0));
      q.coeffs().coeffRef(i) = Scalar(0.5) * t;
      t = Scalar(0.5)/t;
      q.w() = (mat.coeff(k,j)-mat.coeff(j,k))*t;
      q.coeffs().coeffRef(j) = (mat.coeff(j,i)+mat.coeff(i,j))*t;
      q.coeffs().coeffRef(k) = (mat.coeff(k,i)+mat.coeff(i,k))*t;
    }
  }
};