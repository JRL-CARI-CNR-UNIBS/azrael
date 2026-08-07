from dataclasses import dataclass

@dataclass
class ParameterTypes:
    BOOL         : int = 0
    INT          : int = 1
    FLOAT        : int = 2
    STRING       : int = 3
    BOOL_ARRAY   : int = 4
    INT_ARRAY    : int = 5
    FLOAT_ARRAY  : int = 6
    STRING_ARRAY : int = 7

    @staticmethod
    def as_string(p):
        match(p):
            case ParameterTypes.BOOL:
                return "bool"
            case ParameterTypes.INT:
                return "int"
            case ParameterTypes.FLOAT:
                return "float"
            case ParameterTypes.STRING:
                return "string"
            case ParameterTypes.BOOL_ARRAY:
                return "bool_array"
            case ParameterTypes.INT_ARRAY:
                return "int_array"
            case ParameterTypes.FLOAT_ARRAY:
                return "float_array"
            case ParameterTypes.STRING_ARRAY:
                return "string_array"
            case _:
                raise ValueError(f"{p} is not a valid type")
