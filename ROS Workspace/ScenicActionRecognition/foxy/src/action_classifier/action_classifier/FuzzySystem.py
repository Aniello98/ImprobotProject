import skfuzzy as fuzz
import numpy as np
from skfuzzy import control as ctrl
from skfuzzy.control.fuzzyvariable import FuzzyVariable
from typing import List

EPS = 0.0001

'''
NEW: ACTIONS have been modified in the merging process of input classification and output generation
OLD_ACTIONS = [
    "attack", #attack
    "intimidation", #intimidate
    "refuse", #integrated with disappointment
    "grudge", #grudge
    "scolding", #scolding
    "disappointment", #disappointment 
    "sad_person", #integrated with share sadness
    "share_sadness", #sharing_sadness
    "share_fear", #sharing fear
    "share_surprise", #surprise
    "share_joy", #sharing_happiness
    "greet", #integrated in sharing_happiness
    "happiness_person", #happy
    "satisfaction", #satisfaction 
    "caution", #integrated in share fear
    "disbelief", #disbelief
    "hesitancy", #integrated in share fear
    "perplexity", #integrated in share surprise
    "astonishment", #astonishment
    "shock", #integrated in share fear
    "escape", #running_away
    "none"
]
'''

ACTIONS = [
    "attack", #attack
    "intimidation", #intimidate
    "refuse", 
    "grudge", #grudge
    "scolding", #scolding
    "disappointment", #disappointment 
    "sad_person", 
    "share_sadness", #sharing_sadness
    "share_fear", #sharing fear
    "share_surprise", #surprise
    "share_joy", #sharing_happiness
    "greet",
    "happiness_person", #happy
    "satisfaction", #satisfaction 
    "caution", 
    "disbelief", #disbelief
    "hesitancy",
    "perplexity",
    "astonishment", #astonishment
    "shock",
    "escape", #running_away
    "none"
]

INTEGRATED_ACTIONS = {
    "attack" : "attack", 
    "intimidation" : "intimidate",
    "refuse" : "disappointment", 
    "grudge" : "grudge", 
    "scolding" : "scolding", 
    "disappointment" : "disappointment",  
    "sad_person" : "sharing_sadness", 
    "share_sadness" : "sharing_sadness", 
    "share_fear" : "sharing_fear", 
    "share_surprise" : "surprise", 
    "share_joy" : "sharing_happiness", 
    "greet" : "sharing_happiness",
    "happiness_person" : "happy_person", 
    "satisfaction" : "satisfaction",  
    "caution" : "sharing_fear", 
    "disbelief" : "disbelief", 
    "hesitancy" : "sharing_fear", 
    "perplexity" : "surprise", 
    "astonishment" : "astonishment", 
    "shock" : "sharing_fear", 
    "escape" : "running_away", 
    "none" : "none"
}

EMOTIONS = ["anger",
            "fear",
            "disgust", #not in claudia's code
            "happiness", #joy
            "sadness",
            "surprise",
            "neutral"]

NEUTRAL_INDEX = ACTIONS.index("none")


def create_singleton(variable: FuzzyVariable, names: List[str]) -> None:
    for i, name in enumerate(names):
        variable[name] = fuzz.trimf(variable.universe, [i - EPS, i, i + EPS])


class FuzzySystem:

    __control_system: ctrl.ControlSystem
    __matrix: ctrl.ControlSystemSimulation

    __rules: List

    def add_printer(self, printer):
        self.printer = printer

    def __init__(self) -> None:
        # Initialize fuzzy variables
        # Proxemic Zone
        zone = ctrl.Antecedent(
            np.arange(0, 3, EPS),
            "zone"
        )
        create_singleton(zone, ["int", "neu", "not"])
        # Proxemic Movement
        lin_mov = ctrl.Antecedent(
            np.arange(-1, 1, EPS),
            "lin_mov"
        )
        # Define fuzzy MFs for speed
        lin_mov["still"] = fuzz.trapmf(
            lin_mov.universe, [-0.02, -0.01, 0.01, 0.02])
        lin_mov["app_slow"] = fuzz.trapmf(
            lin_mov.universe, [-0.07, -0.06, -0.02, -0.01])
        lin_mov["app_med"] = fuzz.trapmf(
            lin_mov.universe, [-0.14, -0.13, -0.07, -0.06])
        lin_mov["app_fast"] = fuzz.trapmf(
            lin_mov.universe, [-1, -1, -0.14, -0.13])

        lin_mov["ret_slow"] = fuzz.trapmf(
            lin_mov.universe, [0.015, 0.02, 0.06, 0.065])
        lin_mov["ret_med"] = fuzz.trapmf(
            lin_mov.universe, [0.06, 0.07, 0.13, 0.14])
        lin_mov["ret_fast"] = fuzz.trapmf(
            lin_mov.universe, [0.13, 0.14, 1, 1])

        lin_mov.view()

        # Emotions
        emotion = ctrl.Antecedent(
            np.arange(0, 8, 0.01),
            "emotion"
        )
        # Define linear maps for emotions
        emotion["anger"] = fuzz.trimf(emotion.universe, [0, 1, 1])
        emotion["fear"] = fuzz.trimf(emotion.universe, [1, 2, 2])
        emotion["disgust"] = fuzz.trimf(emotion.universe, [2, 3, 3])
        emotion["happiness"] = fuzz.trimf(emotion.universe, [3, 4, 4])
        emotion["sadness"] = fuzz.trimf(emotion.universe, [4, 5, 5])
        emotion["surprise"] = fuzz.trimf(emotion.universe, [5, 6, 6])
        emotion["neutral"] = fuzz.trimf(emotion.universe, [6, 7, 7])

        # Create output variable for scenic actions
        action = ctrl.Consequent(np.arange(0, len(ACTIONS), 0.01), "action")
        create_singleton(action, ACTIONS)

        # Initialize the self.__rules
        self.__rules = []
        # ===== ANGER =====
        self.__rules.append(
            ctrl.Rule(
                lin_mov["app_fast"] & emotion["anger"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["attack"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                zone["int"] & emotion["anger"] & (lin_mov["still"] | lin_mov["app_slow"] | lin_mov["app_med"] | lin_mov["app_fast"] | lin_mov["ret_slow"] |lin_mov["ret_med"] |lin_mov["ret_fast"] ),
                action["attack"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                (lin_mov["app_slow"] | lin_mov["app_med"] | lin_mov["still"]) & (
                    zone["int"] | zone["neu"]) & emotion["anger"],
                action["intimidation"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                lin_mov["still"] & ~zone["int"] & emotion["anger"],
                action["scolding"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                ((lin_mov["ret_slow"] | lin_mov["ret_med"]
                    | lin_mov["ret_fast"]) & (zone["neu"] | zone["not"])) & emotion["anger"],
                action["grudge"]  # TODO: Here understand if zone is relevant
            )
        )
        self.__rules.append(
            ctrl.Rule(
                emotion["happiness"] & zone["int"] & (lin_mov["still"]),
                action["share_joy"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                emotion["happiness"] & ~zone["int"] & lin_mov["still"],
                action["happiness_person"]
            )
        )
        
        self.__rules.append(
            ctrl.Rule(
                #emotion["happiness"] & (zone["neu"] | (~zone["not"] & (lin_mov["app_slow"] | lin_mov["app_med"] | lin_mov["app_fast"]))),
                emotion["happiness"] & (~zone["not"] & (lin_mov["app_slow"] | lin_mov["app_med"])),
                action["greet"]
            )
        )
        
        self.__rules.append(
            ctrl.Rule(
                #(lin_mov["ret_slow"] | lin_mov["ret_med"] | lin_mov["ret_fast"]
                (lin_mov["ret_slow"] 
                 ) & ~zone["int"] & emotion["happiness"],
                action["satisfaction"]
            )
        )
        # ===== SADNESS =====
        self.__rules.append(
            ctrl.Rule(
                #(lin_mov["ret_slow"] | lin_mov["ret_med"] | lin_mov["ret_fast"]
                (lin_mov["ret_slow"] | lin_mov["ret_med"] 
                 ) & emotion["sadness"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["disappointment"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                zone["int"] & emotion["sadness"] & (lin_mov["still"] ),
                action["share_sadness"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                #~zone["int"] & ~(lin_mov["ret_slow"] | lin_mov["ret_med"] | lin_mov["ret_fast"]
                ~zone["int"] & (lin_mov["ret_slow"] | lin_mov["still"] | lin_mov["app_slow"]
                                 ) & emotion["sadness"],
                action["sad_person"]
            )
        )
        # ===== SURPRISE =====
        self.__rules.append(
            ctrl.Rule(
                ~zone["not"] & emotion["surprise"] & (lin_mov["still"] | lin_mov["ret_slow"] ),
                action["share_surprise"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                #(lin_mov["still"] | lin_mov["app_med"] |
                #lin_mov["app_slow"]) & ~zone["int"] & emotion["surprise"],
                lin_mov["still"]  & ~zone["int"] & emotion["surprise"],
                action["astonishment"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                #(lin_mov["ret_fast"] | lin_mov["ret_slow"] |lin_mov["ret_med"])
                  lin_mov["ret_slow"] & emotion["surprise"] & (zone["int"] | zone["neu"] ),
                action["disbelief"]
            )
        )
        
        # ===== FEAR =====
        self.__rules.append(
            ctrl.Rule(
                lin_mov["still"] & zone["not"] & emotion["fear"],
                action["shock"]
            )
        )
 
        self.__rules.append(
            ctrl.Rule(
                zone["int"] & emotion["fear"] & (lin_mov["still"] | lin_mov["ret_slow"] ),
                action["share_fear"]
            )
        )

        self.__rules.append(
            ctrl.Rule(
                (lin_mov["app_fast"] | lin_mov["app_med"] | lin_mov["app_slow"] |
                 lin_mov["still"]) & ~zone["not"] & emotion["fear"],
                action["share_fear"]
            )
        )

        self.__rules.append(
            ctrl.Rule(
                lin_mov["app_slow"] & ~zone["int"] & emotion["fear"],
                action["caution"]
            )
        )

        self.__rules.append(
            ctrl.Rule(
                (lin_mov["still"] | lin_mov["app_slow"] |
                 lin_mov["ret_slow"]) & ~zone["int"] & emotion["fear"],
                action["hesitancy"]
            )
        )

        self.__rules.append(
            ctrl.Rule(
                (lin_mov["ret_fast"] | lin_mov["ret_med"]) &
                emotion["fear"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["escape"]
            )
        )

        self.__rules.append(
            ctrl.Rule(
                lin_mov["ret_slow"] &
                emotion["fear"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["shock"]
            )
        )

        # ===== DISGUST =====
        
        self.__rules.append(
            ctrl.Rule(
                (lin_mov["ret_slow"] | lin_mov["ret_med"] | lin_mov["ret_fast"]) &
                emotion["disgust"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["refuse"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                lin_mov["still"] &
                emotion["disgust"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["hesitancy"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                (lin_mov["app_slow"] | lin_mov["app_med"] | lin_mov["app_fast"]) &
                emotion["disgust"] & (zone["int"] | zone["neu"] | zone["not"]),
                action["perplexity"]
            )
        )
        self.__rules.append(
            ctrl.Rule(
                (zone["int"] | zone["neu"] | zone["not"]) & (lin_mov["still"] | lin_mov["app_slow"] | lin_mov["app_med"] | lin_mov["app_fast"] | lin_mov["ret_slow"] |lin_mov["ret_med"] |lin_mov["ret_fast"]) &
                emotion["neutral"],
                action["none"]
            )
        )

        self.__control_system = ctrl.ControlSystem(self.__rules)
        self.__matrix = ctrl.ControlSystemSimulation(self.__control_system)

    def __map_emotion_vector_to_fuzzy(self, emotion_vector: np.ndarray) -> float:
        """Map the provided vector of confidence scores for emotions into the corresponding value of the emotion fuzzy variable 

        Args:
            emotion_vector (np.ndarray): an array containing the confidence scores for each emotion

        Returns:
            float: the value of the fuzzy variable on the universe of discourse
        """
        # Retrieve index of dominant emotion
        i = np.argmax(emotion_vector)
        self.printer.print_msg(f"dominant: {EMOTIONS[i]}")
        # And correposnding confidence value
        dominant_score = emotion_vector[i]
        return i + dominant_score

    def classify(self, zone, lin_mov, emotion_vector) -> int:
        self.__matrix.input["zone"] = zone
        self.printer.print_msg(f"Zone: {zone}")
        emotion_value = self.__map_emotion_vector_to_fuzzy(
            emotion_vector)
        # We use the emotion vector as is, and map it onto the emotion variable
        self.__matrix.input["emotion"] = emotion_value
        self.__matrix.input["lin_mov"] = lin_mov
        self.printer.print_msg(f"Movement: {lin_mov}")
        # Compute the simulation
        self.__matrix.compute()
        # Return the defuzzified result (the encoding of classified action)
        result = self.__matrix.output["action"]
        return result

    def get_label(self, action_index: int) -> str:
        # NEW: return the action considering the new integration done
        action = ACTIONS[action_index]
        return INTEGRATED_ACTIONS[action]

    def get_rules(self):
        return self.__rules
