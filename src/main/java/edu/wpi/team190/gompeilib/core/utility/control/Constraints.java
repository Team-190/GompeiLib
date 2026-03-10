package edu.wpi.team190.gompeilib.core.utility.control;

import java.util.function.Consumer;

public sealed interface Constraints<T extends Constraints<T>>
    permits AngularConstraints, LinearConstraints {
  public void update(int id, Consumer<T> consumer);
}
