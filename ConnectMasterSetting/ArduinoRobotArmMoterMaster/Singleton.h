#ifndef _SINGLETON_H
#define _SINGLETON_H

namespace tmlib {

/**
    @brief  シングルトンホルダー
*/
template <class _Ty>
class SingletonHolder
{
  public :
    typedef _Ty InstanceType;

  public :
    /// 生成
    static void create()
    {
      if (_instance == NULL) {
        _instance = new InstanceType();
      }
    }
    /// 破棄
    static void destroy()
    {
      if (_instance != NULL) {
        delete _instance;
        _instance = NULL;
      }
    }

#if 1
    /// インスタンスゲット
    static InstanceType& getInstance()
    {
      return *_instance;
    }
#else
    /// インスタンスゲット
    static InstanceType* getInstance()
    {
      return _instance;
    }
#endif
    /// インスタンス参照ゲット
    static InstanceType& getInstanceRef()
    {
      return *_instance;
    }

    /// インスタンスポインタゲット
    static InstanceType* getInstancePtr()
    {
      return _instance;
    }

    /// 生成済み??
    static bool isCreate()  {
      return _instance != NULL;
    };
    /// 破棄済み??
    static bool isDestroy() {
      return _instance == NULL;
    };

  private :
    static InstanceType* _instance; //!< インスタンス

  private :
    // 禁止
    SingletonHolder() {}
    virtual ~SingletonHolder() {}

    SingletonHolder(const SingletonHolder&);
    SingletonHolder& operator=(const SingletonHolder&);
};
template <class _Ty>
typename SingletonHolder<_Ty>::InstanceType* SingletonHolder<_Ty>::_instance = NULL;

}
#endif // _SINGLETON_H
