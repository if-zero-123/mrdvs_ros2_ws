#ifndef LIO_UPDATE_GUARD_H_
#define LIO_UPDATE_GUARD_H_

namespace fast_livo
{

inline bool hasUsableLioConstraints(int effective_feature_num)
{
  return effective_feature_num > 0;
}

} // namespace fast_livo

#endif // LIO_UPDATE_GUARD_H_
