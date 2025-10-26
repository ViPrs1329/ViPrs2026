This is the code for the 2026 FRC Robotics competition for the ViPrs

# Coding Practices

Let's use a standard way of programming to avoid confusion

### 1. Camel Case

> Camel case is a common naming convention that uses capital letters to denote new words in a variable name instead of underscores.
> Ironically, this means that we will not use snake case in our code. For example, `myFavoriteVariable` instead of `my_favorite_variable`.

### 2. Type Safety

> Let's try to use type hints whenever possible when naming variables. For example:
>
> ```py
> myFavoriteVariable: str = "Hello World"
> ```
> 
> instead of
> 
> ```py
> myFavoriteVariable = "Hello World"
> ```
>
> This makes debugging code way easier since when you hover over the variable, it will show what type it is supposed to be.
>
> Also, functions should have a return type as seen below:
>
> ```py
> def myFunc() -> None:
>     # stuff in here
> ```

### 3. Docstrings

> When defining functions, there should be a little blurb about what the function does (arguments and return values are optional). For example:
>
> ```py
> def myFunc(messageToPrint: str):
>     """
>     Prints out a message
> 
>     :param messageToPrint: the message to be printed
>     """
>     print(messageToPrint)
> ```
>
> Like with the type hints, these help with debugging, and they make the code more readable to someone who doesn't know what a function does.

### 4. Comments

> This might be one of the most important things to do, especially on larger projects around 5,000 lines, like a robot. Comments are denoted with the pound key (#) and are brief descriptions of what is happening in the code. These help when someone else, or even you, is reviewing your code. You right now, and you in two weeks, are two different people. Here is an example:
>
> ```py
> # score a coral on L1 on the left branch of the reef face
> L1Command: SequentialCommandGroup = SequentialCommandGroup(
>     ParallelCommandGroup(
>         InstantCommand(
>             lambda: AutoAlign(constants.Field.ReefLeftPosition),
>             Drivetrain
>         ),
>         InstantCommand(
>             ElevatorGoTo(constants.Elevator.Heights.L1),
>             Elevator
>         )
>     ),
>     ScoreCoralCommand
> )
> ```
>
> As you can see, it is not always obvious when looking at some code to see what it does.

# Using Github

Here at ViPrs, we use a software called Github. It is a place where we can store all of our code in what are called "repositories". This ensures that it is kept safe from corruption and is accessible from anywhere.

### Why Use Github?

We use Github as a place to store the code for the robot since it provides safe storage (free from possible corruption), version management, and easy accessibility.

### How to Use Branches

Branches are for pulling a copy of the actual robot code and changing it without messing with the real robot code. The actual code is stored on the "main" branch, and variations of it are stored in other branches. The way that we will use branches is by following these steps: 

1. Make a branch off of an existing branch. This is usually the "main" branch.
2. Add changes to the branch you have just created. This is where you will program, test, and push code.
3. Merge the one you just created into the original branch to append your changes to it. This will add your changes to the "main" branch.
4. Remove the branch that you have created to not clutter the branches. This ensures that the menu containing the branches isn't filled with 100 branches.

### Commits

Every time you commit some code to a branch when editing an existing branch or creating a new branch, Github will automatically creates a version of the code and stores it so you can either revert to a previous version if you messed it up, or go back and view what was there before.

---

Let's make this season our best season

# Let's Go ViPrs!
