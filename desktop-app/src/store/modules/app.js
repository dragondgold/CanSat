const state = {
  sidebar: {
    opened: false,
    withoutAnimation: false
  },
  config: {
    googleMaps: {
      apiKey: 'AIzaSyBNAqPrTQoz9P4NBlDDyfxrnKiafkaL8iQ'
    },
    windowMatchSizeLg: '(min-width: 992px)',
    palette: {
      primary: '#4ae387',
      danger: '#e34a4a',
      info: '#4ab2e3',
      success: '#db76df',
      warning: '#f7cc36',
      white: '#fff',
      black: '#000',
      fontColor: '#34495e',
      transparent: 'transparent',
      lighterGray: '#ddd'
    }
  },
  isLoading: true,
  axtec: {
    project:{
      path:'',
      cansat:[{
        id: '',
        name:'',
        status:'',
        protocol: '',
        sensors: [{
          type: '',
          status:'',
          samplingTime: '',
          samples: [{
            value: '',
            timespan:''
          }],
          threshold: ''
        }],
        actuators: [{
          type:'',
          status: ''
        }],
        protections: {
          powerSupply: [{
            voltage: '',
            current: '',
            status: '',
            maxCurrent: '',
          }]
        }
      }],
      groundStation: {
        id: '',
        status: '' 
      }
    },
    data:{

    }
  }
}

const mutations = {
  setLoading (state, isLoading) {
    state.isLoading = isLoading
  },
  'TOGGLE_WITHOUT_ANIMATION' (state, value) {
    state.sidebar.withoutAnimation = value
  },
  axtecPath(state, path){
    state.axtec.project.path = path
  }
}

const actions = {
  isToggleWithoutAnimation ({ commit }, value) {
    commit('TOGGLE_WITHOUT_ANIMATION', value)
  },
  axtecPath( { commit }, value){
    commit(axtecPath,value)
  }
}

export default {
  state,
  mutations,
  actions
}