package ca.mcgill.ecse211.navigation;

/**
 * This class is used to handle errors regarding the singleton pattern used for the navigation
 *
 */
@SuppressWarnings("serial")
public class NavigationExceptions extends Exception {

  public NavigationExceptions(String Error) {
    super(Error);
  }

}
