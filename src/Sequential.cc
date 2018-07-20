#include "Sequential.h"
#include <iostream>
using namespace  std;
namespace ORB_SLAM2 {
Sequential::Queue<int> Sequential::_LocalOptQ;
Sequential::Queue<int> Sequential::_LoopClosingQ;
bool Sequential::_playNFFlag=true;
void Sequential::localMappingClear(){
    cout<<"|||||| Sequential::localMappingClear"<<endl;
    _LocalOptQ.clear();
}
  void Sequential::waitForEndLocalMapping(){
      cout<<"|||||| Sequential::waitForEndLocalMapping"<<endl;
    int v;
    _LocalOptQ.pop();
}
  void Sequential::endLocalMapping(){
      cout<<"|||||| Sequential::endLocalMapping"<<endl;
    _LocalOptQ.push(1);
}
  void Sequential::loopClosingClear(){
      cout<<"|||||| Sequential::loopClosingClear"<<endl;
      _LoopClosingQ.clear();
  }
  void Sequential::endLoopClosing(){
      cout<<"|||||| Sequential::endLoopClosing"<<endl;

    _LoopClosingQ.push(1);
}
  void Sequential::waitForEndLoopClosing(){
      cout<<"|||||| Sequential::waitForEndLoopClosing"<<endl;

    int v;
    _LoopClosingQ.pop();
}

}
