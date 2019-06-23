#pragma once

#include <memory>

namespace remote_control {

/** Forward declaration of the RemoteControl class with type aliases.
 **
 ** If the RemoteControl class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class RemoteControl;

    using RemoteControlPtr = std::shared_ptr<RemoteControl>;
    using RemoteControlConstPtr = std::shared_ptr<const RemoteControl>;
}
